"""Main entry point for Drone GCS"""

import sys
import logging
import os

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import QApplication

from drone_gcs.ui import MainWindow


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    app = QApplication(sys.argv)
    app.setApplicationName("Drone GCS")
    app.setOrganizationName("DroneFirmware")

    QTimer.singleShot(0, lambda: _apply_dark_theme(app))

    window = MainWindow()
    window.show()

    return app.exec()


def _apply_dark_theme(app: QApplication) -> None:
    dark_palette = _get_kde_colors()

    app.setStyle("Fusion")
    app.setPalette(dark_palette)

    app.setStyleSheet("""
        QGroupBox {
            font-weight: bold;
            border: 1px solid palette(mid);
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 8px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 8px;
            padding: 0 4px;
        }
        QPushButton {
            padding: 6px 16px;
            min-width: 70px;
            border: 1px solid palette(mid);
            border-radius: 4px;
        }
        QPushButton:hover {
            background-color: palette(highlight);
        }
        QPushButton:pressed {
            background-color: palette(dark);
        }
        QSpinBox, QDoubleSpinBox {
            padding: 4px;
            border: 1px solid palette(mid);
            border-radius: 3px;
        }
        QComboBox {
            padding: 4px 8px;
            border: 1px solid palette(mid);
            border-radius: 3px;
        }
        QTabWidget::pane {
            border: 1px solid palette(mid);
            border-radius: 4px;
        }
        QTabBar::tab {
            padding: 6px 12px;
            border: 1px solid palette(mid);
            border-radius: 3px;
        }
        QTabBar::tab:selected {
            background-color: palette(highlight);
        }
        QProgressBar {
            border: 1px solid palette(mid);
            border-radius: 4px;
            text-align: center;
        }
        QProgressBar::chunk {
            background-color: palette(highlight);
            border-radius: 3px;
        }
        QStatusBar {
            border-top: 1px solid palette(mid);
        }
        QMenuBar {
            border-bottom: 1px solid palette(mid);
        }
        QMenuBar::item:selected {
            background-color: palette(highlight);
        }
        QMenu {
            border: 1px solid palette(mid);
        }
        QMenu::item:selected {
            background-color: palette(highlight);
        }
        QTextEdit {
            border: 1px solid palette(mid);
            border-radius: 3px;
            background-color: palette(base);
        }
        QLineEdit {
            padding: 4px 8px;
            border: 1px solid palette(mid);
            border-radius: 3px;
        }
    """)


def _get_kde_colors() -> "QPalette":
    try:
        from PyQt6.QtGui import QPalette, QColor

        if os.environ.get("KDE_FULL_SESSION") or os.environ.get("KDE_SESSION_VERSION"):
            try:
                from PyQt6.QtGui import QGuiApplication
                from PyQt6.QtCore import QLibraryInfo

                palette = QGuiApplication.palette()
                return palette
            except Exception:
                pass

        base_colors = {
            "breeze": {
                "Window": QColor(35, 35, 35),
                "WindowText": QColor(239, 239, 239),
                "Base": QColor(45, 45, 45),
                "AlternateBase": QColor(55, 55, 55),
                "Text": QColor(239, 239, 239),
                "Button": QColor(60, 60, 60),
                "ButtonText": QColor(239, 239, 239),
                "BrightText": QColor(239, 239, 239),
                "Link": QColor(58, 142, 218),
                "LinkVisited": QColor(88, 160, 220),
                "Highlight": QColor(58, 142, 218),
                "HighlightedText": QColor(239, 239, 239),
                "Mid": QColor(80, 80, 80),
                "Light": QColor(70, 70, 70),
                "Dark": QColor(25, 25, 25),
            }
        }

        palette = QPalette()
        colors = base_colors["breeze"]

        palette.setColor(QPalette.ColorRole.Window, colors["Window"])
        palette.setColor(QPalette.ColorRole.WindowText, colors["WindowText"])
        palette.setColor(QPalette.ColorRole.Base, colors["Base"])
        palette.setColor(QPalette.ColorRole.AlternateBase, colors["AlternateBase"])
        palette.setColor(QPalette.ColorRole.Text, colors["Text"])
        palette.setColor(QPalette.ColorRole.Button, colors["Button"])
        palette.setColor(QPalette.ColorRole.ButtonText, colors["ButtonText"])
        palette.setColor(QPalette.ColorRole.BrightText, colors["BrightText"])
        palette.setColor(QPalette.ColorRole.Link, colors["Link"])
        palette.setColor(QPalette.ColorRole.Highlight, colors["Highlight"])
        palette.setColor(QPalette.ColorRole.HighlightedText, colors["HighlightedText"])
        palette.setColor(QPalette.ColorRole.Mid, colors["Mid"])
        palette.setColor(QPalette.ColorRole.Light, colors["Light"])
        palette.setColor(QPalette.ColorRole.Dark, colors["Dark"])

        for role in [
            QPalette.ColorRole.Window,
            QPalette.ColorRole.Base,
            QPalette.ColorRole.Button,
        ]:
            brush = palette.brush(role)
            brush.setColor(
                colors["Base"] if role == QPalette.ColorRole.Base else colors["Window"]
            )

        return palette

    except Exception as e:
        logging.warning(f"Could not get KDE colors: {e}")
        from PyQt6.QtGui import QPalette, QColor

        return QPalette(QColor(35, 35, 35), QColor(45, 45, 45))


if __name__ == "__main__":
    sys.exit(main())
