#!/usr/bin/env python3
"""TASC 2027 judge console (operator side).

Judge interface for CIRC 2027 Automated Demolition and Clearing
(https://circ.cstag.ca/2027/tasks/). One labelled button per operation,
labelled "[shortname] long name" with the short names exactly as the rules
write them. One click (plus the text boxes the rules call for) = one operation.

Stage 1: no ROS. A click builds a Command and hands it to a CommandSink.
PrintSink prints it to the terminal; the on-screen log shows the same line.
Phases 1-2 (drive) are live. Phases 3-7 are shown greyed out for design review.

Run:       python3 judge_console.py
Self-test: python3 judge_console.py --selftest   (works offscreen)
"""

import sys
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

# ----------------------------------------------------------------------------
# Team settings (not on the judge screen: the judge gives one input only)
# ----------------------------------------------------------------------------

# Drive to Waypoint / Drive to Marked Waypoint use obstacle avoidance.
# The obstacle-free route scores half points, so this stays on unless the
# team decides otherwise before the run.
OBSTACLE_AVOIDANCE = True


# ----------------------------------------------------------------------------
# Operation model
# ----------------------------------------------------------------------------

@dataclass(frozen=True)
class Field:
    name: str          # key used in Command.args and in the phrase template
    label: str         # text shown next to the box
    unit: str          # unit shown next to the box (rules: units must be shown)
    placeholder: str = ''
    lo: Optional[float] = None
    hi: Optional[float] = None
    positive: bool = False   # must be > 0
    integer: bool = False


@dataclass(frozen=True)
class Operation:
    op_id: str        # machine id sent with the command
    short: str        # official short name from the rules, shown in [brackets]
    long: str         # long name
    phrase: str       # printed/logged text; {field} is replaced by the value
    fields: Tuple[Field, ...] = ()
    returns: str = ''  # key in RETURNED if this operation returns a value
    live: bool = True  # False = shown greyed out, not wired yet
    phases: str = ''   # mission phases that use it (tooltip only)

    @property
    def label(self) -> str:
        return f'[{self.short}] {self.long}'


@dataclass
class Command:
    op_id: str
    args: Dict[str, float]
    phrase: str
    stamp: str


# Returned values and the units they are shown in.
RETURNED = {
    'bearing': 'Bearing (°, true north = 0)',
    'gnss': 'GNSS (WGS 84 decimal degrees, lat, lon)',
    'markers': 'Stored marker IDs (left to right)',
    'eff': 'EFF position (m, from indicator package)',
}

# --- fields ---------------------------------------------------------------

LAT = Field('lat', 'Lat', '°', '51.46', -90, 90)
LON = Field('lon', 'Lon', '°', '-112.71', -180, 180)

# --- Operations, grouped by what they do. Each appears once. -----------------
# Later phases reuse earlier commands, so phases are only shown in tooltips.

DRIVE = [
    *[Operation(f'drive_forward_{n}', 'Drive Forward', f'Drive forward {n} m',
                f'going forward {n} m', phases='1+') for n in (1, 3, 5)],
    Operation('drive_backwards', 'Drive Backwards', 'Drive backwards N m',
              'going backwards {distance} m',
              (Field('distance', 'N', 'm', 'e.g. 2', positive=True),), phases='1+'),
    Operation('rotate', 'Rotate', 'Rotate N° (+ = clockwise)',
              'rotating {angle}° from current heading (+ = clockwise)',
              (Field('angle', 'N', '°', 'e.g. -45', -360, 360),), phases='1+'),
    Operation('rotate_and_drive', 'Rotate and Drive', 'Drive N m at M° clockwise',
              'driving {distance} m at {angle}° clockwise from current heading',
              (Field('distance', 'N', 'm', 'e.g. 3', positive=True),
               Field('angle', 'M', '° cw', '0-360', 0, 360)), phases='1+'),
    Operation('face_bearing', 'Face Bearing', 'Face absolute bearing N°',
              'facing absolute bearing {bearing}°',
              (Field('bearing', 'N', '° (N = 0)', '0-360', 0, 360),), phases='1+'),
]

NAVIGATE = [
    Operation('drive_to_waypoint', 'Drive to Waypoint', '3 m from GNSS point',
              'driving to GNSS waypoint {lat}, {lon} (within 3 m)', (LAT, LON),
              phases='2+'),
    Operation('drive_to_marked_waypoint', 'Drive to Marked Waypoint',
              '1 m from ArUco #3 tower',
              'driving to ArUco #3 tower near {lat}, {lon} (within 1 m)', (LAT, LON),
              phases='2+'),
]

REPORT = [
    Operation('report_bearing', 'Report Bearing', 'Current bearing',
              'requesting current bearing', returns='bearing', phases='1+'),
    Operation('report_gnss', 'Report GNSS', 'Current GNSS position',
              'requesting current GNSS position', returns='gnss', phases='1+'),
    Operation('eff_report', 'EFF Report', 'End effector position',
              'requesting end effector position', returns='eff', live=False,
              phases='5'),
]

MARKERS = [
    Operation('read_and_store_markers', 'Read and Store Markers',
              'Read and show board IDs',
              'reading and storing job markers', returns='markers', live=False,
              phases='3'),
    Operation('retrieve_markers', 'Retrieve Markers', 'Return stored IDs',
              'retrieving stored marker IDs', returns='markers', live=False,
              phases='4+'),
]

ARM = [
    Operation('store_home', 'Store Home', 'Stow arm and end effector',
              'storing home: arm and end effector stowed', phases='1, 5'),
    Operation('eff_max_height', 'EFF Max Height', 'Raise to max height',
              'end effector to max height', live=False, phases='5'),
    Operation('eff_min_height', 'EFF Min Height', 'Lower to min height',
              'end effector to min height', live=False, phases='5'),
    Operation('eff_position', 'EFF Position', 'Height and distance',
              'end effector to {height} m high, {distance} m ahead of body front (H = height, D = distance from body front)',
              (Field('height', 'H', 'm', 'e.g. 0.3', lo=0),
               Field('distance', 'D', 'm', 'e.g. 0.2', lo=0)),
              live=False, phases='5'),
    Operation('eff_open', 'EFF Open', 'Open to at least N cm',
              'opening end effector to at least {opening} cm',
              (Field('opening', 'N', 'cm (max 10)', 'e.g. 8', 0, 10, positive=True),),
              live=False, phases='5'),
    Operation('eff_close', 'EFF Close', 'Close completely',
              'closing end effector', live=False, phases='5'),
    Operation('eff_pickup', 'EFF Pickup', 'Pick up at marked spot',
              'picking up object at marked spot', live=False, phases='6'),
    Operation('eff_squeeze', 'EFF Squeeze', 'Close around object or lever',
              'squeezing object, handle or lever', live=False, phases='6'),
    Operation('eff_set_down', 'EFF Set Down', 'Set down at spot',
              'setting object down at specified spot', live=False, phases='6'),
    Operation('eff_press_marked_button', 'EFF Press Marked Button',
              'Button at #N, #M',
              'pressing button located by markers #{marker_n} and #{marker_m}',
              (Field('marker_n', 'N', '', 'ID 0-99', 0, 99, integer=True),
               Field('marker_m', 'M', '', 'ID 0-99', 0, 99, integer=True)),
              live=False, phases='7'),
]

# Two columns on one screen
COLUMNS = [
    [('Drive', DRIVE), ('Navigate', NAVIGATE), ('Report', REPORT)],
    [('Markers', MARKERS), ('Arm and end effector', ARM)],
]

# Team control: not a judged operation, kept as a safety stop.
STOP = Operation('stop', 'STOP', 'Team safety stop, halt all motion',
                 'STOP: halting all motion')


# ----------------------------------------------------------------------------
# Command sinks. Swap PrintSink for a ROS 2 sink in the simulation step.
# ----------------------------------------------------------------------------

class CommandSink:
    def send(self, command: Command) -> None:
        raise NotImplementedError


class PrintSink(CommandSink):
    """Stage 1: print the command. No robot involved."""

    def send(self, command: Command) -> None:
        print(f'[{command.stamp}] {command.op_id} {command.args}: {command.phrase}',
              flush=True)


# ----------------------------------------------------------------------------
# Widgets
# ----------------------------------------------------------------------------

def fmt(value: float) -> str:
    """1.0 -> '1', 2.5 -> '2.5'."""
    return f'{value:g}'


class OperationRow(QWidget):
    """One button, its text boxes (with units) and an attempt counter."""

    def __init__(self, op: Operation,
                 submit: Callable[['OperationRow', Dict[str, float]], None],
                 error: Callable[[str], None]):
        super().__init__()
        self.op = op
        self._submit = submit
        self._error = error
        self.attempts = 0
        self.inputs: List[Tuple[Field, QLineEdit]] = []

        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)

        self.button = QPushButton(op.label)
        self.button.setMinimumHeight(38)
        tip = op.phrase + (f'\nUsed in phase {op.phases}' if op.phases else '')
        if not op.live:
            tip += '\nNot wired yet'
        self.button.setToolTip(tip)
        self.button.setEnabled(op.live)
        self.button.clicked.connect(self._on_click)
        row.addWidget(self.button)

        for f in op.fields:
            row.addWidget(QLabel(f.label))
            edit = QLineEdit()
            edit.setPlaceholderText(f.placeholder)
            edit.setFixedWidth(64)
            edit.setEnabled(op.live)
            edit.returnPressed.connect(self.button.click)
            row.addWidget(edit)
            if f.unit:
                row.addWidget(QLabel(f.unit))
            self.inputs.append((f, edit))

        self.badge = QLabel('')
        self.badge.setMinimumWidth(30)
        self.badge.setToolTip('Attempts (first attempt scores full points)')
        row.addWidget(self.badge)
        row.addStretch()

    def _on_click(self) -> None:
        args: Dict[str, float] = {}
        for f, edit in self.inputs:
            text = edit.text().strip()
            try:
                value = float(text)
            except ValueError:
                self._error(f'[{self.op.short}] {f.label} needs a number in {f.unit}')
                return
            if f.integer and value != int(value):
                self._error(f'[{self.op.short}] {f.label} must be a whole number')
                return
            if f.positive and value <= 0:
                self._error(f'[{self.op.short}] {f.label} must be greater than 0 {f.unit}')
                return
            if (f.lo is not None and value < f.lo) or (f.hi is not None and value > f.hi):
                self._error(f'[{self.op.short}] {f.label} must be between '
                            f'{fmt(f.lo)} and {fmt(f.hi)} {f.unit}')
                return
            args[f.name] = int(value) if f.integer else value
        self.attempts += 1
        self.badge.setText(f'×{self.attempts}')
        self._submit(self, args)


class JudgeConsole(QMainWindow):
    def __init__(self, sink: CommandSink):
        super().__init__()
        self.sink = sink
        self.rows: List[OperationRow] = []
        self.returned: Dict[str, QLabel] = {}
        self.setWindowTitle('TASC 2027 Judge Console: Automated Demolition and Clearing')

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Top bar
        top = QHBoxLayout()
        title = QLabel('Automated Demolition and Clearing')
        title.setObjectName('title')
        top.addWidget(title)
        top.addStretch()
        top.addWidget(QLabel(f'Output: {type(sink).__name__}   |   Obstacle avoidance: '
                             f'{"ON" if OBSTACLE_AVOIDANCE else "OFF (half points)"} '
                             f'(team setting)'))
        stop_btn = QPushButton(STOP.label)
        stop_btn.setObjectName('stop')
        stop_btn.setToolTip('Team control, not a judged operation')
        stop_btn.clicked.connect(self._stop)
        top.addWidget(stop_btn)
        root.addLayout(top)

        # Autonomy Guidelines: teams declare entering autonomous mode, then no team
        # member may give ANY input to the operations computer until it is stopped.
        auto_bar = QHBoxLayout()
        self.auto_btn = QPushButton('Declare autonomous mode')
        self.auto_btn.setObjectName('auto')
        self.auto_btn.setCheckable(True)
        self.auto_btn.toggled.connect(self._set_autonomous)
        self.auto_banner = QLabel('')
        self.auto_banner.setObjectName('banner')
        auto_bar.addWidget(self.auto_btn)
        auto_bar.addWidget(self.auto_banner, 1)
        root.addLayout(auto_bar)

        # Operations: two columns, grouped by function, no tabs
        ops_area = self._columns(COLUMNS)

        # Right side: returned values, team log, log
        side = QWidget()
        side.setMaximumWidth(330)
        side_layout = QVBoxLayout(side)

        values_box = QGroupBox('Returned values')
        values_layout = QVBoxLayout(values_box)
        for key, title_text in RETURNED.items():
            head = QLabel(title_text)
            head.setObjectName('valuehead')
            value = QLabel('no value yet')
            value.setObjectName('value')
            value.setWordWrap(True)
            values_layout.addWidget(head)
            values_layout.addWidget(value)
            self.returned[key] = value
        side_layout.addWidget(values_box)

        team_box = QGroupBox('Team log (locked in autonomous mode)')
        team_layout = QHBoxLayout(team_box)
        reset_btn = QPushButton('Log reset')
        reset_btn.setToolTip('Rover was physically reset to its starting pose (no penalty)')
        reset_btn.clicked.connect(lambda: self.log('TEAM: rover reset to starting pose'))
        skip_btn = QPushButton('Log skip / intervention')
        skip_btn.setToolTip('Rover moved to a later start point (modification intervention)')
        skip_btn.clicked.connect(lambda: self.log('TEAM: modification intervention, '
                                                  'rover moved to another start point'))
        team_layout.addWidget(reset_btn)
        team_layout.addWidget(skip_btn)
        side_layout.addWidget(team_box)
        self.team_buttons = [reset_btn, skip_btn]

        log_box = QGroupBox('Log')
        log_layout = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        log_layout.addWidget(self.log_view)
        side_layout.addWidget(log_box, 1)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(ops_area)
        splitter.addWidget(side)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([1360, 320])
        root.addWidget(splitter, 1)

        self._set_autonomous(False, quiet=True)
        self.log('Console ready. Drive, Navigate, Report Bearing/GNSS and Store Home are live; '
                 'greyed-out buttons are not wired yet.')

    # ---- layout --------------------------------------------------------------

    def _columns(self, columns) -> QWidget:
        inner = QWidget()
        layout = QHBoxLayout(inner)
        for groups in columns:
            col = QVBoxLayout()
            col_rows = []
            for title, ops in groups:
                box = QGroupBox(title)
                box_layout = QVBoxLayout(box)
                box_layout.setSpacing(4)
                for op in ops:
                    row = OperationRow(op, self._submit, self._error)
                    self.rows.append(row)
                    col_rows.append(row)
                    box_layout.addWidget(row)
                col.addWidget(box)
            col.addStretch()
            # same button width within a column, wide enough for the longest label
            width = max(r.button.sizeHint().width() for r in col_rows) + 8
            for r in col_rows:
                r.button.setFixedWidth(width)
            layout.addLayout(col)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(inner)
        return scroll

    # ---- actions -------------------------------------------------------------

    def _submit(self, row: OperationRow, args: Dict[str, float]) -> None:
        op = row.op
        phrase = op.phrase.format(**{k: fmt(v) for k, v in args.items()})
        if op.op_id in ('drive_to_waypoint', 'drive_to_marked_waypoint'):
            args = {**args, 'obstacle_avoidance': 1.0 if OBSTACLE_AVOIDANCE else 0.0}
        command = Command(op.op_id, args, phrase, datetime.now().strftime('%H:%M:%S'))
        note = '' if self.autonomous else '  (WARNING: autonomous mode not declared)'
        self.log(f'[{op.short}] attempt {row.attempts}: {phrase}{note}')
        self.sink.send(command)
        if op.returns:
            self.set_returned(op.returns, 'requested, waiting for rover')

    def _set_autonomous(self, on: bool, quiet: bool = False) -> None:
        self.autonomous = on
        for b in getattr(self, 'team_buttons', []):
            b.setEnabled(not on)
        if on:
            self.auto_btn.setText('End autonomous mode')
            self.auto_banner.setText('AUTONOMOUS MODE: hands off. Judge input only. '
                                     'Any team input here ends autonomy.')
            self.auto_banner.setProperty('on', True)
        else:
            self.auto_btn.setText('Declare autonomous mode')
            self.auto_banner.setText('Not autonomous. Declare to the judge before operations.')
            self.auto_banner.setProperty('on', False)
        self.auto_banner.style().unpolish(self.auto_banner)
        self.auto_banner.style().polish(self.auto_banner)
        if not quiet:
            self.log('TEAM: autonomous mode declared' if on else 'TEAM: autonomous mode ended')

    def _stop(self) -> None:
        if self.autonomous:
            self.auto_btn.setChecked(False)   # team input: autonomy ends
        command = Command(STOP.op_id, {}, STOP.phrase, datetime.now().strftime('%H:%M:%S'))
        self.log(STOP.phrase)
        self.sink.send(command)

    def _error(self, message: str) -> None:
        self.log(f'NOT SENT: {message}')

    # ---- hooks for the ROS step ------------------------------------------------

    def set_returned(self, key: str, text: str) -> None:
        """Show a returned value. Text must include its units."""
        self.returned[key].setText(text)

    def show_bearing(self, degrees: float) -> None:
        self.set_returned('bearing', f'{degrees % 360:.1f}°')

    def show_gnss(self, lat: float, lon: float) -> None:
        self.set_returned('gnss', f'{lat:.6f}°, {lon:.6f}°')

    def show_markers(self, ids: List[int]) -> None:
        self.set_returned('markers', '  '.join(f'#{i}' for i in ids))

    def show_eff(self, x: float, y: float, z: float) -> None:
        self.set_returned('eff', f'x {x:.3f} m, y {y:.3f} m, z {z:.3f} m')

    def log(self, message: str) -> None:
        self.log_view.appendPlainText(f'{datetime.now().strftime("%H:%M:%S")}  {message}')


# ----------------------------------------------------------------------------
# Entry points
# ----------------------------------------------------------------------------

STYLE = """
QMainWindow, QWidget { background: #14171c; color: #e6e9ef; font-size: 13px; }
QScrollArea { border: none; }
QGroupBox { border: 1px solid #2c323c; border-radius: 6px; margin-top: 14px; padding: 8px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; color: #9fb3c8; }
QPushButton { background: #223043; border: 1px solid #365074; border-radius: 6px;
              padding: 6px 12px; text-align: left; }
QPushButton:hover { background: #2b3d56; }
QPushButton:disabled { background: #1b2028; color: #5d6670; border-color: #272d36; }
QPushButton#stop { background: #5a1f22; border-color: #8a3137; font-weight: bold; }
QLineEdit { background: #0e1115; border: 1px solid #2c323c; border-radius: 4px; padding: 4px; }
QLineEdit:disabled { color: #5d6670; }
QPlainTextEdit { background: #0e1115; border: 1px solid #2c323c; font-family: monospace; }
QPushButton#auto { background: #1f3a2a; border-color: #2f6b45; font-weight: bold; }
QPushButton#auto:checked { background: #2f6b45; }
QLabel#banner { padding: 6px 10px; border-radius: 4px; background: #2a2416; color: #e7c46a; }
QLabel#banner[on="true"] { background: #173524; color: #7fe0a4; font-weight: bold; }
QLabel#title { font-size: 16px; font-weight: bold; }
QLabel#valuehead { color: #9fb3c8; font-size: 12px; }
QLabel#value { font-size: 15px; font-weight: bold; padding-bottom: 6px; }
"""

SAMPLES = {'distance': '2', 'angle': '90', 'bearing': '50',
           'lat': '51.46', 'lon': '-112.71'}


def selftest(window: JudgeConsole) -> int:
    """Fill sample values, click every live button, check validation."""
    window.auto_btn.setChecked(True)
    locked = not any(b.isEnabled() for b in window.team_buttons)
    live = [r for r in window.rows if r.op.live]
    for row in live:
        for f, edit in row.inputs:
            edit.setText(SAMPLES[f.name])
        row.button.click()
    sent = sum(r.attempts for r in live)

    # validation: these must NOT send
    back = next(r for r in live if r.op.op_id == 'drive_backwards')
    face = next(r for r in live if r.op.op_id == 'face_bearing')
    before = back.attempts + face.attempts
    back.inputs[0][1].setText('0'); back.button.click()
    back.inputs[0][1].setText('abc'); back.button.click()
    face.inputs[0][1].setText('400'); face.button.click()
    blocked = (back.attempts + face.attempts) == before

    # every label uses an official short name in [brackets]
    labels_ok = all(r.button.text().startswith(f'[{r.op.short}] ') for r in window.rows)

    window.show_bearing(271.46)
    window.show_gnss(51.4612345, -112.7098765)
    window._stop()
    ended = not window.autonomous and all(b.isEnabled() for b in window.team_buttons)
    print(f'selftest: {len(live)} live buttons, {sent} sent, '
          f'bad input blocked={blocked}, labels ok={labels_ok}, '
          f'team log locked in auto={locked}, STOP ends auto={ended}')
    return 0 if (sent == len(live) and blocked and labels_ok and locked and ended) else 1


def main() -> int:
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    window = JudgeConsole(PrintSink())
    window.resize(1680, 900)
    window.show()
    if '--selftest' in sys.argv:
        code = selftest(window)
        app.quit()
        return code
    return app.exec()


if __name__ == '__main__':
    sys.exit(main())
