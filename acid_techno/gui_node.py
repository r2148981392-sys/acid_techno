import sys
from dataclasses import dataclass

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.interpolate import griddata
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from PySide6.QtWidgets import QGridLayout
from PySide6.QtWidgets import QLabel
from PySide6.QtWidgets import QMainWindow
from PySide6.QtWidgets import QSizePolicy
from PySide6.QtWidgets import QVBoxLayout
from PySide6.QtWidgets import QWidget


GRID_SIZE_X = 3.0
GRID_SIZE_Y = 3.0
CELL_SIZE = 0.3
PH_MIN = 6.0
PH_MAX = 9.0


@dataclass
class GridStateSnapshot:
    rows: int
    cols: int
    values: np.ndarray


def grid_extent(rows: int, cols: int) -> tuple[float, float, float, float]:
    half_x = cols * CELL_SIZE / 2.0
    half_y = rows * CELL_SIZE / 2.0
    return (-half_x, half_x, -half_y, half_y)


def cell_center(row: int, col: int, rows: int, cols: int) -> tuple[float, float]:
    x = (col + 0.5) * CELL_SIZE - (cols * CELL_SIZE) / 2.0
    y = (row + 0.5) * CELL_SIZE - (rows * CELL_SIZE) / 2.0
    return x, y


def parse_grid_state(message: Float64MultiArray) -> GridStateSnapshot | None:
    if len(message.data) < 2:
        return None

    rows = int(round(message.data[0]))
    cols = int(round(message.data[1]))
    if rows <= 0 or cols <= 0:
        return None

    expected = rows * cols
    if len(message.data) < 2 + expected:
        return None

    values = np.array(message.data[2:2 + expected], dtype=float).reshape((rows, cols))
    return GridStateSnapshot(rows=rows, cols=cols, values=values)


def measured_sample_points(grid_state: GridStateSnapshot) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sample_x = []
    sample_y = []
    sample_ph = []

    for row in range(grid_state.rows):
        for col in range(grid_state.cols):
            ph_value = grid_state.values[row, col]
            if PH_MIN <= ph_value <= PH_MAX:
                x, y = cell_center(row, col, grid_state.rows, grid_state.cols)
                sample_x.append(x)
                sample_y.append(y)
                sample_ph.append(ph_value)

    return np.array(sample_x, dtype=float), np.array(sample_y, dtype=float), np.array(sample_ph, dtype=float)


def generate_acidity_map_from_grid(grid_state: GridStateSnapshot, resolution: int = 200) -> tuple[np.ndarray, tuple[float, float, float, float], str]:
    sample_x, sample_y, sample_ph = measured_sample_points(grid_state)
    extent = grid_extent(grid_state.rows, grid_state.cols)

    if sample_ph.size < 3:
        measured = np.full((grid_state.rows, grid_state.cols), np.nan, dtype=float)
        for row in range(grid_state.rows):
            for col in range(grid_state.cols):
                ph_value = grid_state.values[row, col]
                if PH_MIN <= ph_value <= PH_MAX:
                    measured[row, col] = ph_value
        return measured, extent, 'Waiting for more samples for interpolation'

    grid_x, grid_y = np.mgrid[
        extent[0]:extent[1]:complex(resolution),
        extent[2]:extent[3]:complex(resolution),
    ]
    interpolated = griddata(
        (sample_x, sample_y),
        sample_ph,
        (grid_x, grid_y),
        method='linear',
    )

    if interpolated is None:
        return np.full((grid_state.rows, grid_state.cols), np.nan, dtype=float), extent, 'Waiting for more samples for interpolation'

    if np.isnan(interpolated).any():
        nearest = griddata(
            (sample_x, sample_y),
            sample_ph,
            (grid_x, grid_y),
            method='nearest',
        )
        interpolated = np.where(np.isnan(interpolated), nearest, interpolated)

    return interpolated.T, extent, 'Interpolated acidity map'


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.latest_ph: float | None = None
        self.latest_x: float | None = None
        self.latest_y: float | None = None
        self.grid_state: GridStateSnapshot | None = None
        self.grid_message: str = 'Waiting for data...'

        self.create_subscription(Float64, '/acidity', self.acidity_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/grid_state', self.grid_state_callback, 10)

    def acidity_callback(self, message: Float64):
        self.latest_ph = float(message.data)

    def odom_callback(self, message: Odometry):
        self.latest_x = float(message.pose.pose.position.x)
        self.latest_y = float(message.pose.pose.position.y)

    def grid_state_callback(self, message: Float64MultiArray):
        snapshot = parse_grid_state(message)
        if snapshot is None:
            return

        self.grid_state = snapshot
        measured_count = int(np.sum((snapshot.values >= PH_MIN) & (snapshot.values <= PH_MAX)))
        accessible_count = int(np.sum(snapshot.values != -2.0))
        if accessible_count > 0 and measured_count >= accessible_count:
            self.grid_message = 'Scan status: complete'
        else:
            self.grid_message = 'Scan status: scanning'


class MappingWindow(QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.acidity_colorbar = None

        self.setWindowTitle('Acid Techno Mapping GUI')

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        root_layout = QVBoxLayout(central_widget)

        header_layout = QGridLayout()
        self.ph_label = QLabel('Current pH: waiting for data')
        self.x_label = QLabel('Robot x: waiting for data')
        self.y_label = QLabel('Robot y: waiting for data')
        self.measured_label = QLabel('Measured grid squares: 0')
        self.scan_label = QLabel('Scan status: waiting for data')

        header_layout.addWidget(self.ph_label, 0, 0)
        header_layout.addWidget(self.x_label, 0, 1)
        header_layout.addWidget(self.y_label, 1, 0)
        header_layout.addWidget(self.measured_label, 1, 1)
        header_layout.addWidget(self.scan_label, 2, 0, 1, 2)
        root_layout.addLayout(header_layout)

        status_layout = QGridLayout()
        self.acidity_status_label = QLabel('Acidity status: waiting for grid state')
        self.grid_status_label = QLabel('Grid status: waiting for grid state')
        status_layout.addWidget(self.acidity_status_label, 0, 0)
        status_layout.addWidget(self.grid_status_label, 0, 1)
        root_layout.addLayout(status_layout)

        self.figure = Figure(figsize=(12, 6), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        root_layout.addWidget(self.canvas)

        self.acidity_ax = self.figure.add_subplot(1, 2, 1)
        self.grid_ax = self.figure.add_subplot(1, 2, 2)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_ros)
        self.timer.start(50)

        self.refresh_labels()
        self.redraw()

    def poll_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.refresh_labels()
        self.redraw()

    def refresh_labels(self):
        if self.node.latest_ph is None:
            self.ph_label.setText('Current pH: waiting for data')
        else:
            self.ph_label.setText(f'Current pH: {self.node.latest_ph:.2f}')

        if self.node.latest_x is None:
            self.x_label.setText('Robot x: waiting for data')
        else:
            self.x_label.setText(f'Robot x: {self.node.latest_x:.2f} m')

        if self.node.latest_y is None:
            self.y_label.setText('Robot y: waiting for data')
        else:
            self.y_label.setText(f'Robot y: {self.node.latest_y:.2f} m')

        if self.node.grid_state is None:
            self.measured_label.setText('Measured grid squares: 0')
            self.scan_label.setText('Scan status: waiting for data')
            self.acidity_status_label.setText('Acidity status: waiting for grid state')
            self.grid_status_label.setText('Grid status: waiting for grid state')
            return

        measured_count = int(np.sum((self.node.grid_state.values >= PH_MIN) & (self.node.grid_state.values <= PH_MAX)))
        self.measured_label.setText(f'Measured grid squares: {measured_count}')
        self.scan_label.setText(self.node.grid_message)

    def redraw(self):
        if self.acidity_colorbar is not None:
            self.acidity_colorbar.remove()
            self.acidity_colorbar = None

        self.acidity_ax.clear()
        self.grid_ax.clear()

        if self.node.grid_state is None:
            self.draw_empty_panel(self.acidity_ax, 'Waiting for grid state')
            self.draw_empty_panel(self.grid_ax, 'Waiting for grid state')
            self.canvas.draw_idle()
            return

        self.draw_acidity_map(self.acidity_ax)
        self.draw_grid_map(self.grid_ax)
        self.canvas.draw_idle()

    def draw_empty_panel(self, axis, message: str):
        axis.set_xlim(-GRID_SIZE_X / 2.0, GRID_SIZE_X / 2.0)
        axis.set_ylim(-GRID_SIZE_Y / 2.0, GRID_SIZE_Y / 2.0)
        axis.set_xticks(np.arange(-GRID_SIZE_X / 2.0, GRID_SIZE_X / 2.0 + CELL_SIZE, CELL_SIZE), minor=True)
        axis.set_yticks(np.arange(-GRID_SIZE_Y / 2.0, GRID_SIZE_Y / 2.0 + CELL_SIZE, CELL_SIZE), minor=True)
        axis.grid(which='minor', color='0.85', linestyle='-', linewidth=0.8)
        axis.grid(which='major', visible=False)
        axis.text(0.5, 0.5, message, transform=axis.transAxes, ha='center', va='center')
        axis.set_title(message)
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')

    def draw_acidity_map(self, axis):
        assert self.node.grid_state is not None
        acidity_data, extent, message = generate_acidity_map_from_grid(self.node.grid_state)

        self.acidity_status_label.setText(f'Acidity status: {message}')

        image = axis.imshow(
            acidity_data,
            extent=extent,
            origin='lower',
            cmap='gray',
            vmin=PH_MIN,
            vmax=PH_MAX,
            aspect='equal',
        )
        axis.set_title('Acidity map')
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_xticks(np.arange(extent[0], extent[1] + CELL_SIZE, CELL_SIZE), minor=True)
        axis.set_yticks(np.arange(extent[2], extent[3] + CELL_SIZE, CELL_SIZE), minor=True)
        axis.grid(which='minor', color='white', linestyle='-', linewidth=0.4, alpha=0.25)
        axis.grid(which='major', visible=False)
        self.acidity_colorbar = self.figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
        self.acidity_colorbar.set_label('pH')

        if self.node.latest_x is not None and self.node.latest_y is not None:
            axis.scatter([self.node.latest_x], [self.node.latest_y], c='black', s=40, marker='o', edgecolors='white', linewidths=0.8)

    def draw_grid_map(self, axis):
        assert self.node.grid_state is not None

        extent = grid_extent(self.node.grid_state.rows, self.node.grid_state.cols)
        measured_values = np.full((self.node.grid_state.rows, self.node.grid_state.cols), np.nan, dtype=float)
        overlay_values = np.full((self.node.grid_state.rows, self.node.grid_state.cols), np.nan, dtype=float)

        measured_count = 0
        for row in range(self.node.grid_state.rows):
            for col in range(self.node.grid_state.cols):
                value = self.node.grid_state.values[row, col]
                if value == -2.0:
                    overlay_values[row, col] = 1.0
                elif value == -1.0:
                    overlay_values[row, col] = 0.0
                elif PH_MIN <= value <= PH_MAX:
                    measured_values[row, col] = value
                    measured_count += 1

        self.grid_status_label.setText(f'Grid status: Measured cells: {measured_count}')

        ph_image = axis.imshow(
            measured_values,
            extent=extent,
            origin='lower',
            cmap='viridis',
            vmin=PH_MIN,
            vmax=PH_MAX,
            aspect='equal',
        )

        overlay_cmap = ListedColormap([
            (0.85, 0.85, 0.85, 0.45),
            (0.75, 0.10, 0.10, 0.65),
        ])
        axis.imshow(
            overlay_values,
            extent=extent,
            origin='lower',
            cmap=overlay_cmap,
            vmin=-0.5,
            vmax=1.5,
            aspect='equal',
        )

        axis.set_title('Grid square map')
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_xticks(np.arange(extent[0], extent[1] + CELL_SIZE, CELL_SIZE), minor=True)
        axis.set_yticks(np.arange(extent[2], extent[3] + CELL_SIZE, CELL_SIZE), minor=True)
        axis.grid(which='minor', color='black', linestyle='-', linewidth=0.45, alpha=0.25)
        axis.grid(which='major', visible=False)

        if self.node.latest_x is not None and self.node.latest_y is not None:
            axis.scatter([self.node.latest_x], [self.node.latest_y], c='white', s=55, marker='x', linewidths=1.6)

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()

    app = QApplication(sys.argv)
    window = MappingWindow(node)
    window.resize(1400, 800)
    window.show()

    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    main()