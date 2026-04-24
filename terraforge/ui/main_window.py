import logging
import os
import sys

from PyQt6.QtCore import QThread, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from terraforge.cli import run_generate_world
from terraforge.utils.logging import setup_logger

logger = setup_logger('terraforge.gui', log_level=logging.DEBUG)


class WorldGeneratorThread(QThread):
    generation_started = pyqtSignal()
    generation_progress = pyqtSignal(str)
    generation_percent = pyqtSignal(int)
    generation_finished = pyqtSignal(str)
    generation_error = pyqtSignal(str)
    generation_cancelled = pyqtSignal()

    def __init__(self, latitude, longitude, radius, output_dir, world_name):
        super().__init__()
        self.latitude = latitude
        self.longitude = longitude
        self.radius = radius
        self.output_dir = output_dir
        self.world_name = world_name
        self._cancel = False

    def cancel(self):
        """Request cancellation. The pipeline polls this between stages
        and raises InterruptedError from run_generate_world, which we
        catch below and emit as generation_cancelled."""
        self._cancel = True

    def run(self):
        self.generation_started.emit()
        try:
            world_path = run_generate_world(
                latitude=self.latitude,
                longitude=self.longitude,
                radius=self.radius,
                output_dir=self.output_dir,
                world_name=self.world_name,
                progress=self.generation_progress.emit,
                progress_percent=self.generation_percent.emit,
                cancel_flag=lambda: self._cancel,
            )
            self.generation_finished.emit(world_path)
        except InterruptedError:
            self.generation_cancelled.emit()
        except Exception as e:
            logger.exception("World generation failed")
            self.generation_error.emit(str(e))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TerraForge Gazebo World Builder")
        self.resize(640, 520)

        central = QWidget(self)
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        form = QFormLayout()
        self.latitudeLineEdit = QLineEdit("37.7749")
        self.longitudeLineEdit = QLineEdit("-122.4194")
        self.radiusLineEdit = QLineEdit("1000")
        self.worldNameLineEdit = QLineEdit("generated_world")
        self.outputDirLineEdit = QLineEdit(os.path.abspath('generated_worlds_gui'))
        form.addRow("Latitude:", self.latitudeLineEdit)
        form.addRow("Longitude:", self.longitudeLineEdit)
        form.addRow("Radius (m):", self.radiusLineEdit)
        form.addRow("World name:", self.worldNameLineEdit)

        out_row = QHBoxLayout()
        out_row.addWidget(self.outputDirLineEdit)
        self.browseOutputDirButton = QPushButton("Browse...")
        out_row.addWidget(self.browseOutputDirButton)
        form.addRow("Output directory:", out_row)

        root.addLayout(form)

        button_row = QHBoxLayout()
        self.generateWorldButton = QPushButton("Generate World")
        self.cancelButton = QPushButton("Cancel")
        self.cancelButton.setEnabled(False)
        button_row.addWidget(self.generateWorldButton)
        button_row.addWidget(self.cancelButton)
        root.addLayout(button_row)

        self.progressBar = QProgressBar()
        self.progressBar.setRange(0, 100)
        root.addWidget(self.progressBar)

        self.logPlainTextEdit = QPlainTextEdit()
        self.logPlainTextEdit.setReadOnly(True)
        root.addWidget(self.logPlainTextEdit)

        self.browseOutputDirButton.clicked.connect(self.browse_output_directory)
        self.generateWorldButton.clicked.connect(self.start_world_generation)
        self.cancelButton.clicked.connect(self.cancel_world_generation)

        self.world_gen_thread = None

    @pyqtSlot()
    def browse_output_directory(self):
        dialog = QFileDialog()
        dialog.setFileMode(QFileDialog.FileMode.Directory)
        dialog.setOption(QFileDialog.Option.ShowDirsOnly, True)
        output_dir = dialog.getExistingDirectory(
            self, "Select Output Directory", self.outputDirLineEdit.text()
        )
        if output_dir:
            self.outputDirLineEdit.setText(output_dir)

    @pyqtSlot()
    def start_world_generation(self):
        try:
            latitude = float(self.latitudeLineEdit.text())
            longitude = float(self.longitudeLineEdit.text())
            radius = float(self.radiusLineEdit.text())
        except ValueError:
            QMessageBox.warning(
                self, "Input Error",
                "Please enter valid numeric values for Latitude, Longitude, and Radius.",
            )
            return

        world_name = self.worldNameLineEdit.text().strip()
        output_dir = self.outputDirLineEdit.text().strip()
        if not world_name:
            QMessageBox.warning(self, "Warning", "World name cannot be empty.")
            return
        if not output_dir:
            QMessageBox.warning(self, "Warning", "Output directory cannot be empty.")
            return

        os.makedirs(output_dir, exist_ok=True)

        self.generateWorldButton.setEnabled(False)
        self.cancelButton.setEnabled(True)
        self.progressBar.setValue(0)
        self.logPlainTextEdit.clear()

        self.world_gen_thread = WorldGeneratorThread(
            latitude, longitude, radius, output_dir, world_name
        )
        self.world_gen_thread.generation_started.connect(self.on_generation_started)
        self.world_gen_thread.generation_progress.connect(self.on_generation_progress)
        self.world_gen_thread.generation_percent.connect(self.on_generation_percent)
        self.world_gen_thread.generation_finished.connect(self.on_generation_finished)
        self.world_gen_thread.generation_error.connect(self.on_generation_error)
        self.world_gen_thread.generation_cancelled.connect(self.on_generation_cancelled)
        self.world_gen_thread.start()

    @pyqtSlot()
    def cancel_world_generation(self):
        if self.world_gen_thread is not None and self.world_gen_thread.isRunning():
            self.logPlainTextEdit.appendPlainText("Cancellation requested — "
                                                  "finishing current stage then aborting.")
            self.world_gen_thread.cancel()
            # Button stays enabled for a moment in case the user wants to click
            # again; the thread's final emit of generation_cancelled resets it.
            self.cancelButton.setEnabled(False)

    def on_generation_started(self):
        self.progressBar.setValue(5)

    def on_generation_progress(self, message):
        self.logPlainTextEdit.appendPlainText(message)

    def on_generation_percent(self, percent):
        self.progressBar.setValue(int(percent))

    def _reset_buttons(self):
        self.generateWorldButton.setEnabled(True)
        self.cancelButton.setEnabled(False)

    def on_generation_finished(self, world_path):
        self.progressBar.setValue(100)
        self._reset_buttons()
        QMessageBox.information(
            self, "Success", f"Gazebo world generated successfully:\n{world_path}"
        )

    def on_generation_error(self, error_message):
        self.progressBar.setValue(0)
        self._reset_buttons()
        QMessageBox.critical(self, "Error", f"World generation failed:\n{error_message}")
        self.logPlainTextEdit.appendPlainText(f"Error: {error_message}")

    def on_generation_cancelled(self):
        self.progressBar.setValue(0)
        self._reset_buttons()
        self.logPlainTextEdit.appendPlainText("World generation cancelled.")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
