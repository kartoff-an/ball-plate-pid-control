import sys
import logging

from PyQt6.QtWidgets import QApplication

from main_window import BallPlateWindow


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)-8s | %(name)s | %(message)s",
        datefmt="%H:%M:%S",
        handlers=[logging.StreamHandler(sys.stdout)],
    )
    app = QApplication(sys.argv)
    app.setApplicationName("Ball and Plate Control System")
    app.setStyle("Fusion")

    window = BallPlateWindow()
    window.showMaximized()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()