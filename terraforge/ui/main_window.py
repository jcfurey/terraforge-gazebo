import json
import logging
import os
import shutil
import sys

import shapely.geometry
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

from terraforge.data_acquisition import elevation, osm, textures
from terraforge.data_processing import (
    building_processor,
    elevation_processor,
    texture_processor,
)
from terraforge.data_processing.sdf_builder import SDFWorldBuilder
from terraforge.utils.config import config
from terraforge.utils.coordinates import CoordinateConverter
from terraforge.utils.logging import setup_logger

logger = setup_logger('terraforge.gui', log_level=logging.DEBUG)

TEMPLATE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'data_processing', 'templates',
)


class WorldGeneratorThread(QThread):
    generation_started = pyqtSignal()
    generation_progress = pyqtSignal(str)
    generation_finished = pyqtSignal(str)
    generation_error = pyqtSignal(str)

    def __init__(self, latitude, longitude, radius, output_dir, world_name):
        super().__init__()
        self.latitude = latitude
        self.longitude = longitude
        self.radius = radius
        self.output_dir = output_dir
        self.world_name = world_name

    def run(self):
        self.generation_started.emit()
        origin_location = (self.latitude, self.longitude)
        location_name = f"loc_{self.latitude:.4f}_{self.longitude:.4f}"

        self.generation_progress.emit("Starting Data Acquisition...")
        dem_output_path = os.path.join(
            config.DEM_OUTPUT_DIR, f"{location_name}_dem.tif"
        )
        osm_output_path = os.path.join(
            config.OSM_OUTPUT_DIR, f"{location_name}_buildings.geojson"
        )
        texture_output_dir = os.path.join(
            config.TEXTURE_OUTPUT_DIR, f"{location_name}_texture"
        )
        os.makedirs(texture_output_dir, exist_ok=True)

        try:
            elevation.download_dem(origin_location, self.radius, dem_output_path)
            self.generation_progress.emit("DEM data downloaded.")
            osm.download_osm_buildings(origin_location, self.radius, osm_output_path)
            self.generation_progress.emit("OSM building data downloaded.")
            textures.download_satellite_texture_tiles(
                origin_location, self.radius, texture_output_dir,
                mapbox_api_key=config.MAPBOX_API_KEY,
            )
            self.generation_progress.emit("Satellite textures downloaded.")
        except Exception as e:
            error_msg = f"Data acquisition failed: {e}"
            logger.error(error_msg)
            self.generation_error.emit(error_msg)
            return

        self.generation_progress.emit("Starting Data Processing...")
        heightmap_output_path = os.path.join(
            config.DEM_OUTPUT_DIR, f"{location_name}_heightmap.png"
        )
        building_sdf_output_dir = os.path.join(
            config.OSM_OUTPUT_DIR, f"{location_name}_building_models_sdf"
        )
        processed_texture_output_dir = os.path.join(
            config.TEXTURE_OUTPUT_DIR, "processed_textures"
        )
        processed_texture_output_path = os.path.join(
            processed_texture_output_dir, "satellite_texture.png"
        )

        try:
            elevation_processor.process_dem_to_heightmap(
                dem_output_path, heightmap_output_path
            )
            self.generation_progress.emit("DEM processed to heightmap.")
            building_processor.process_osm_buildings_to_sdf(
                osm_output_path, building_sdf_output_dir
            )
            self.generation_progress.emit("OSM buildings processed to SDF models.")
            texture_processor.process_satellite_texture(
                texture_output_dir, processed_texture_output_path
            )
            self.generation_progress.emit("Satellite texture processed.")
        except Exception as e:
            error_msg = f"Data processing failed: {e}"
            logger.error(error_msg)
            self.generation_error.emit(error_msg)
            return

        self.generation_progress.emit("Setting up Coordinate Conversion...")
        converter = CoordinateConverter(origin_location)

        self.generation_progress.emit("Starting SDF World Generation...")
        sdf_builder = SDFWorldBuilder(TEMPLATE_DIR)

        building_model_paths = (
            [
                os.path.join(building_sdf_output_dir, f)
                for f in os.listdir(building_sdf_output_dir)
                if f.endswith('.sdf')
            ]
            if os.path.exists(building_sdf_output_dir)
            else []
        )

        building_poses_gazebo = []
        if os.path.exists(osm_output_path):
            with open(osm_output_path, 'r') as f:
                osm_data = json.load(f)
            for feature in osm_data['features']:
                if feature['geometry']['type'] in ('Polygon', 'MultiPolygon'):
                    polygon = shapely.geometry.shape(feature['geometry'])
                    centroid = polygon.centroid
                    gazebo_pose = converter.wgs84_to_gazebo((centroid.y, centroid.x))
                    building_poses_gazebo.append(gazebo_pose[:2])

        output_sdf_world_path = os.path.join(self.output_dir, f"{self.world_name}.world")
        output_textures_dir = os.path.join(
            self.output_dir, "media", "materials", "textures"
        )
        os.makedirs(output_textures_dir, exist_ok=True)

        texture_path_for_sdf = None
        output_texture_file_in_media = os.path.join(
            output_textures_dir, "satellite_texture.png"
        )
        if os.path.exists(processed_texture_output_path):
            shutil.copy2(processed_texture_output_path, output_texture_file_in_media)
            texture_path_for_sdf = os.path.relpath(
                output_texture_file_in_media, os.path.dirname(output_sdf_world_path)
            )

        try:
            sdf_content = sdf_builder.render_world_template(
                heightmap_path=heightmap_output_path,
                texture_path=texture_path_for_sdf,
                building_model_paths=building_model_paths,
                building_poses=building_poses_gazebo,
            )
            sdf_builder.save_sdf_world_file(sdf_content, output_sdf_world_path)
            self.generation_progress.emit("SDF world file generated.")
            self.generation_finished.emit(self.output_dir)
        except Exception as e:
            error_msg = f"SDF world generation failed: {e}"
            logger.error(error_msg)
            self.generation_error.emit(error_msg)
            return


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
        self.browseOutputDirButton = QPushButton("Browse…")
        out_row.addWidget(self.browseOutputDirButton)
        form.addRow("Output directory:", out_row)

        root.addLayout(form)

        self.generateWorldButton = QPushButton("Generate World")
        root.addWidget(self.generateWorldButton)

        self.progressBar = QProgressBar()
        self.progressBar.setRange(0, 100)
        root.addWidget(self.progressBar)

        self.logPlainTextEdit = QPlainTextEdit()
        self.logPlainTextEdit.setReadOnly(True)
        root.addWidget(self.logPlainTextEdit)

        self.browseOutputDirButton.clicked.connect(self.browse_output_directory)
        self.generateWorldButton.clicked.connect(self.start_world_generation)

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
        self.progressBar.setValue(0)
        self.logPlainTextEdit.clear()

        self.world_gen_thread = WorldGeneratorThread(
            latitude, longitude, radius, output_dir, world_name
        )
        self.world_gen_thread.generation_started.connect(self.on_generation_started)
        self.world_gen_thread.generation_progress.connect(self.on_generation_progress)
        self.world_gen_thread.generation_finished.connect(self.on_generation_finished)
        self.world_gen_thread.generation_error.connect(self.on_generation_error)
        self.world_gen_thread.start()

    def on_generation_started(self):
        self.progressBar.setValue(5)

    def on_generation_progress(self, message):
        self.logPlainTextEdit.appendPlainText(message)

    def on_generation_finished(self, output_dir):
        self.progressBar.setValue(100)
        self.generateWorldButton.setEnabled(True)
        QMessageBox.information(
            self, "Success", f"Gazebo world generated successfully in:\n{output_dir}"
        )

    def on_generation_error(self, error_message):
        self.progressBar.setValue(0)
        self.generateWorldButton.setEnabled(True)
        QMessageBox.critical(self, "Error", f"World generation failed:\n{error_message}")
        self.logPlainTextEdit.appendPlainText(f"Error: {error_message}")


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
