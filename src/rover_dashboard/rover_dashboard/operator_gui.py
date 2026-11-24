"""Operator GUI: rover dashboard and judge console as tabs in one window.

CIRC Autonomy Guidelines: during autonomy no one may touch the operations
computer (switching tabs counts) and the judge must see the judge console.
So declaring autonomous mode jumps to the Judge Console tab and locks the
Dashboard tab until autonomy ends (End autonomous mode or STOP).
"""

import sys

from PySide6.QtGui import QFont
from PySide6.QtWidgets import QApplication, QTabWidget

try:
    from .dashboard import DashboardWindow
    from .judge_console import STYLE, JudgeConsole, PrintSink
except ImportError:  # run directly: python3 operator_gui.py
    from dashboard import DashboardWindow  # type: ignore
    from judge_console import STYLE, JudgeConsole, PrintSink  # type: ignore


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName('Rover Dashboard')
    app.setFont(QFont('Arial', 10))

    window = DashboardWindow()
    judge = JudgeConsole(PrintSink())

    tabs = QTabWidget()
    dash_index = tabs.addTab(window.takeCentralWidget(), 'Dashboard')
    tabs.addTab(judge, 'Judge Console')
    window.setCentralWidget(tabs)
    # Scoped to the judge tab so it does not restyle the dashboard.
    judge.setStyleSheet(STYLE)

    def on_autonomy(on: bool) -> None:
        if on:
            tabs.setCurrentWidget(judge)
        # A disabled tab cannot be selected by click or Ctrl+Tab.
        tabs.setTabEnabled(dash_index, not on)
        tabs.setTabToolTip(dash_index, 'Locked during autonomous mode' if on else '')

    # Fires on Declare, End autonomous mode, and STOP (which ends autonomy).
    judge.auto_btn.toggled.connect(on_autonomy)

    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
