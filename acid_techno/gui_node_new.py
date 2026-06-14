import sys
from collections import deque
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from . import grid_model

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.interpolate import griddata
from scipy.interpolate import RBFInterpolator
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

# Switch to the Qt5-compatible Matplotlib backend
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure

# Migrate from PySide6 to PyQt5
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QMainWindow,
    QSizePolicy,
    QVBoxLayout,
    QWidget
)

GRID_SIZE_X = 3.0
GRID_SIZE_Y = 3.0
PH_MIN = 6.0
PH_MAX = 9.0
TEMPERATURE_GRAPH_Y_MIN = 0.0
TEMPERATURE_GRAPH_Y_MAX = 20.0


@dataclass
class GridStateSnapshot:
    rows: int
    cols: int
    values: np.ndarray


def grid_extent(rows: int, cols: int) -> tuple[float, float, float, float]:
    half_x = GRID_SIZE_X / 2.0
    half_y = GRID_SIZE_Y / 2.0
    return (-half_x, half_x, -half_y, half_y)


def cell_center(row: int, col: int, rows: int, cols: int) -> tuple[float, float]:
    cell_size_x = GRID_SIZE_X / cols
    cell_size_y = GRID_SIZE_Y / rows
    x = (col + 0.5) * cell_size_x - GRID_SIZE_X / 2.0
    y = (row + 0.5) * cell_size_y - GRID_SIZE_Y / 2.0
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
        has_any = False
        for row in range(grid_state.rows):
            for col in range(grid_state.cols):
                ph_value = grid_state.values[row, col]
                if PH_MIN <= ph_value <= PH_MAX:
                    measured[row, col] = ph_value
                    has_any = True
        if not has_any:
            return measured, extent, 'Waiting for first sample'
        return measured, extent, 'Waiting for more samples for interpolation'

    grid_x, grid_y = np.mgrid[
        extent[0]:extent[1]:complex(resolution),
        extent[2]:extent[3]:complex(resolution),
    ]
    query_points = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    sample_points = np.column_stack([sample_x, sample_y])

    rbf = RBFInterpolator(
        sample_points,
        sample_ph,
        kernel='thin_plate_spline',
        smoothing=0.1,
    )
    interpolated = np.clip(rbf(query_points).reshape(grid_x.shape), PH_MIN, PH_MAX)

    return interpolated.T, extent, 'Interpolated acidity map'

class GuiNode(Node):
    map: grid_model.MapModel

    def __init__(self):
        super().__init__('gui_node')

        self.latest_ph: float | None = None
        self.latest_x: float | None = None
        self.latest_y: float | None = None
        self.latest_w: float | None = None
        self.latest_temperature: float | None = None
        self.latest_heater_on: bool | None = None
        self.temperature_sample_index = 0
        self.temperature_samples: deque[int] = deque(maxlen=100)
        self.temperature_values: deque[float] = deque(maxlen=100)
        self.current_goal: tuple[float, float] = (0, 0)
        self.grid_state: GridStateSnapshot | None = None
        self.nav_state: GridStateSnapshot | None = None  # ✅ separate nav snapshot
        self.first_goal_sent = False

        self.grid_message: str = 'Waiting for data...'
        self.map: grid_model.MapModel = grid_model.MapModel(GRID_SIZE_X, GRID_SIZE_Y)

        self.create_subscription(Float64, '/acidity', self.acidity_callback, 10)
        #self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub = self.create_subscription(
            Pose2D,
            '/corrected_odom',
            self.odom_callback,
            10
        )
        self.create_subscription(Float64MultiArray, '/grid_state', self.grid_state_callback, 10)
        self.create_subscription(Float64MultiArray, '/nav_state', self.nav_state_callback, 10)  # ✅ new
        self.create_subscription(Float64, '/temperature', self.temperature_callback, 10)
        self.create_subscription(Bool, '/heater_on', self.heater_callback, 10)
        self.goal_sub = self.create_subscription(GoalStatus, '/goal_pub', self.goal_status_callback, 10)

        self.goal_loc_pub = self.create_publisher(PoseStamped, '/goal_location', 10)
        self.grid_state_pub = self.create_publisher(Float64MultiArray, '/grid_state', 10)
        self.nav_state_pub = self.create_publisher(Float64MultiArray, '/nav_state', 10)  # ✅ new
        self.send_goal()

    def send_goal(self):
        if self.latest_x is None or self.latest_y is None or self.latest_w is None:
            return

        self.current_goal = self.map.get_closest_sample_location(self.latest_x, self.latest_y)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(self.current_goal[0])
        pose_msg.pose.position.y = float(self.current_goal[1])
        pose_msg.pose.orientation.w = float(self.latest_w)

        self.goal_loc_pub.publish(pose_msg)

    def goal_status_callback(self, message: GoalStatus):
        # Goal rejected, retry
        if message.status == GoalStatus.STATUS_CANCELED:
            self.send_goal()
            return

        # Goal either successful or unsuccessful, depending on answer edit the map
        if message.status == GoalStatus.STATUS_SUCCEEDED:
            self.map.set_reached(self.current_goal[0], self.current_goal[1])
        elif message.status == GoalStatus.STATUS_ABORTED:
            self.map.mark_location_unreachable(self.current_goal[0], self.current_goal[1])

        self.publish_grid_state()
        self.send_goal()

    def acidity_callback(self, message: Float64):
        if self.latest_x is None or self.latest_y is None:
            return

        self.latest_ph = float(message.data)
        if not self.map.location_measured(self.latest_x, self.latest_y):
            self.map.set_sample(self.latest_x, self.latest_y, self.latest_ph)
            self.publish_grid_state()

    def odom_callback(self, message: Pose2D):
        # self.latest_x = float(message.pose.pose.position.x)
        # self.latest_y = float(message.pose.pose.position.y)
        # self.latest_w = float(message.pose.pose.orientation.w)
        self.latest_x = float(message.x)
        self.latest_y = float(message.y)
        self.latest_w = float(message.theta)

        if not self.first_goal_sent:
            self.first_goal_sent = True
            self.send_goal()

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

    def nav_state_callback(self, message: Float64MultiArray): 
        snapshot = parse_grid_state(message)
        if snapshot is None:
            return
        self.nav_state = snapshot

    def temperature_callback(self, message: Float64):
        self.latest_temperature = float(message.data)
        self.temperature_sample_index += 1
        self.temperature_samples.append(self.temperature_sample_index)
        self.temperature_values.append(self.latest_temperature)

    def heater_callback(self, message: Bool):
        self.latest_heater_on = bool(message.data)

    def publish_grid_state(self):
        # --- Acidity grid (5cm cells) ---
        acidity_msg = Float64MultiArray()
        rows = len(self.map.acidity_grid)
        cols = len(self.map.acidity_grid[0]) if rows > 0 else 0
        data = [float(rows), float(cols)]
        for row in self.map.acidity_grid:
            for square in row:
                data.append(float(square.ph_sample) if square.measured else -1.0)
        acidity_msg.data = data
        self.grid_state_pub.publish(acidity_msg)

        # --- Nav grid (30cm cells) ---
        nav_msg = Float64MultiArray()
        nav_rows = len(self.map.nav_grid)
        nav_cols = len(self.map.nav_grid[0]) if nav_rows > 0 else 0
        nav_data = [float(nav_rows), float(nav_cols)]
        for row in self.map.nav_grid:
            for square in row:
                if not square.accessible:
                    nav_data.append(-2.0)   # unreachable
                elif square.reached:
                    nav_data.append(1.0)    # visited
                else:
                    nav_data.append(-1.0)   # unvisited
        nav_msg.data = nav_data
        self.nav_state_pub.publish(nav_msg)


class MappingWindow(QMainWindow):
    def __init__(self, node: GuiNode):
        super().__init__()
        self.node = node
        self.acidity_colorbar = None
        self.grid_colorbar = None

        self.setWindowTitle('Acid Techno Mapping GUI')

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        root_layout = QVBoxLayout(central_widget)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(12)

        status_row = QHBoxLayout()
        status_row.setSpacing(12)

        sensor_box = QGroupBox('Sensor status')
        sensor_layout = QGridLayout(sensor_box)
        sensor_layout.setContentsMargins(12, 10, 12, 10)
        sensor_layout.setHorizontalSpacing(10)
        sensor_layout.setVerticalSpacing(6)
        self.ph_label = QLabel('Current pH: waiting for data')
        self.acidity_status_label = QLabel('Acidity status: waiting for grid state')
        sensor_layout.addWidget(self.ph_label, 0, 0)
        sensor_layout.addWidget(self.acidity_status_label, 1, 0)

        robot_box = QGroupBox('Robot status')
        robot_layout = QGridLayout(robot_box)
        robot_layout.setContentsMargins(12, 10, 12, 10)
        robot_layout.setHorizontalSpacing(10)
        robot_layout.setVerticalSpacing(6)
        self.x_label = QLabel('Robot x: waiting for data')
        self.y_label = QLabel('Robot y: waiting for data')
        self.scan_label = QLabel('Scan status: waiting for data')
        robot_layout.addWidget(self.x_label, 0, 0)
        robot_layout.addWidget(self.y_label, 1, 0)
        robot_layout.addWidget(self.scan_label, 2, 0)

        grid_box = QGroupBox('Grid status')
        grid_layout = QGridLayout(grid_box)
        grid_layout.setContentsMargins(12, 10, 12, 10)
        grid_layout.setHorizontalSpacing(10)
        grid_layout.setVerticalSpacing(6)
        self.measured_label = QLabel('Measured acidity cells: 0')
        self.grid_status_label = QLabel('Grid status: waiting for grid state')
        grid_layout.addWidget(self.measured_label, 0, 0)
        grid_layout.addWidget(self.grid_status_label, 1, 0)

        status_row.addWidget(sensor_box)
        status_row.addWidget(robot_box)
        status_row.addWidget(grid_box)
        root_layout.addLayout(status_row)

        self.figure = Figure(figsize=(12, 6), constrained_layout=True)
        self.canvas = FigureCanvas(self.figure)

        # Fixed size policy syntax for PyQt5 compatibility
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root_layout.addWidget(self.canvas)

        self.acidity_ax = self.figure.add_subplot(1, 2, 1)
        self.grid_ax = self.figure.add_subplot(1, 2, 2)

        temperature_box = QGroupBox('Temperature / Heater')
        temperature_layout = QVBoxLayout(temperature_box)
        temperature_layout.setContentsMargins(12, 10, 12, 10)
        temperature_layout.setSpacing(8)

        temperature_status_layout = QGridLayout()
        temperature_status_layout.setHorizontalSpacing(10)
        temperature_status_layout.setVerticalSpacing(6)
        self.temperature_label = QLabel('Current temperature: waiting for data')
        self.heater_label = QLabel('Heater status: waiting for data')
        temperature_status_layout.addWidget(self.temperature_label, 0, 0)
        temperature_status_layout.addWidget(self.heater_label, 1, 0)
        temperature_layout.addLayout(temperature_status_layout)

        self.temperature_figure = Figure(figsize=(12, 2.8), constrained_layout=True)
        self.temperature_canvas = FigureCanvas(self.temperature_figure)
        self.temperature_canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.temperature_ax = self.temperature_figure.add_subplot(1, 1, 1)
        temperature_layout.addWidget(self.temperature_canvas)

        root_layout.addWidget(temperature_box)

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
            self.measured_label.setText('Measured acidity cells: 0')
            self.scan_label.setText('Scan status: waiting for data')
            self.acidity_status_label.setText('Acidity map status: waiting for grid state')
            self.grid_status_label.setText('Grid map status: waiting for grid state')
        else:
            measured_count = int(np.sum((self.node.grid_state.values >= PH_MIN) & (self.node.grid_state.values <= PH_MAX)))
            self.measured_label.setText(f'Measured acidity cells: {measured_count}')
            self.scan_label.setText(self.node.grid_message)

        if self.node.latest_temperature is None:
            self.temperature_label.setText('Current temperature: waiting for data')
        else:
            self.temperature_label.setText(f'Current temperature: {self.node.latest_temperature:.2f} °C')

        if self.node.latest_heater_on is None:
            self.heater_label.setText('Heater status: waiting for data')
        else:
            heater_text = 'ON' if self.node.latest_heater_on else 'OFF'
            self.heater_label.setText(f'Heater status: {heater_text}')

    def redraw(self):
        if self.acidity_colorbar is not None:
            self.acidity_colorbar.remove()
            self.acidity_colorbar = None

        self.acidity_ax.clear()
        self.grid_ax.clear()
        self.temperature_ax.clear()

        if self.node.grid_state is None:
            acidity_rows = int(GRID_SIZE_Y / grid_model.AcidityGridSquare.DIMENSION)
            acidity_cols = int(GRID_SIZE_X / grid_model.AcidityGridSquare.DIMENSION)
            self.draw_empty_panel(self.acidity_ax, 'Waiting for acidity data', acidity_rows, acidity_cols)
        else:
            self.draw_acidity_map(self.acidity_ax)

        if self.node.nav_state is None:
            nav_rows = int(GRID_SIZE_Y / grid_model.NavGridSquare.DIMENSION)
            nav_cols = int(GRID_SIZE_X / grid_model.NavGridSquare.DIMENSION)
            self.draw_empty_panel(self.grid_ax, 'Waiting for nav data', nav_rows, nav_cols)
        else:
            self.draw_grid_map(self.grid_ax)

        self.draw_temperature_graph(self.temperature_ax)

        self.canvas.draw_idle()
        self.temperature_canvas.draw_idle()

    def draw_empty_panel(self, axis, message: str, rows: int, cols: int):
        cell_width = GRID_SIZE_X / cols
        cell_height = GRID_SIZE_Y / rows
        axis.set_xlim(-GRID_SIZE_X / 2.0, GRID_SIZE_X / 2.0)
        axis.set_ylim(-GRID_SIZE_Y / 2.0, GRID_SIZE_Y / 2.0)
        axis.set_xticks(np.arange(-GRID_SIZE_X / 2.0, GRID_SIZE_X / 2.0 + cell_width, cell_width), minor=True)
        axis.set_yticks(np.arange(-GRID_SIZE_Y / 2.0, GRID_SIZE_Y / 2.0 + cell_height, cell_height), minor=True)
        axis.grid(which='minor', color='0.85', linestyle='-', linewidth=0.8)
        axis.grid(which='major', visible=False)
        axis.text(0.5, 0.5, message, transform=axis.transAxes, ha='center', va='center')
        axis.set_title(message)
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')

    def draw_acidity_map(self, axis):
        assert self.node.grid_state is not None
        acidity_data, extent, message = generate_acidity_map_from_grid(self.node.grid_state)

        self.acidity_status_label.setText(f'Acidity map status: {message}')

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
        cell_width = GRID_SIZE_X / self.node.grid_state.cols
        cell_height = GRID_SIZE_Y / self.node.grid_state.rows
        axis.set_xticks(np.arange(extent[0], extent[1] + cell_width, cell_width), minor=True)
        axis.set_yticks(np.arange(extent[2], extent[3] + cell_height, cell_height), minor=True)
        axis.grid(which='minor', color='white', linestyle='-', linewidth=0.4, alpha=0.25)
        axis.grid(which='major', visible=False)
        self.acidity_colorbar = self.figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
        self.acidity_colorbar.set_label('pH')

        if self.node.latest_x is not None and self.node.latest_y is not None:
            axis.scatter([self.node.latest_x], [self.node.latest_y], c='black', s=40, marker='o', edgecolors='white', linewidths=0.8)

    def draw_grid_map(self, axis):
        assert self.node.nav_state is not None

        extent = grid_extent(self.node.nav_state.rows, self.node.nav_state.cols)
        status_values = np.zeros((self.node.nav_state.rows, self.node.nav_state.cols), dtype=float)

        visited_count = 0
        for row in range(self.node.nav_state.rows):
            for col in range(self.node.nav_state.cols):
                value = self.node.nav_state.values[row, col]
                if value == -2.0:
                    status_values[row, col] = 2.0   # unreachable
                elif value == 1.0:
                    status_values[row, col] = 1.0   # visited
                    visited_count += 1
                else:
                    status_values[row, col] = 0.0   # unvisited

        self.grid_status_label.setText(f'Grid map status: visited cells {visited_count}')

        status_cmap = ListedColormap([
            (0.88, 0.88, 0.88, 1.0),  # unmeasured
            (0.35, 0.35, 0.35, 1.0),  # measured acidity cell
            (0.05, 0.05, 0.05, 1.0),  # unreachable
        ])
        axis.imshow(
            status_values,
            extent=extent,
            origin='lower',
            cmap=status_cmap,
            vmin=-0.5,
            vmax=2.5,
            aspect='equal',
        )

        axis.set_title('Grid square map')
        axis.set_xlabel('X (m)')
        axis.set_ylabel('Y (m)')
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        cell_width = GRID_SIZE_X / self.node.nav_state.cols
        cell_height = GRID_SIZE_Y / self.node.nav_state.rows
        axis.set_xticks(np.arange(extent[0], extent[1] + cell_width, cell_width), minor=True)
        axis.set_yticks(np.arange(extent[2], extent[3] + cell_height, cell_height), minor=True)
        axis.grid(which='minor', color='black', linestyle='-', linewidth=0.45, alpha=0.25)
        axis.grid(which='major', visible=False)

        if self.node.latest_x is not None and self.node.latest_y is not None:
            axis.scatter([self.node.latest_x], [self.node.latest_y], c='white', s=70, marker='x', linewidths=1.9)

    def draw_temperature_graph(self, axis):
        axis.set_title('Temperature over time')
        axis.set_xlabel('Sample #')
        axis.set_ylabel('Temperature (°C)')
        axis.set_ylim(TEMPERATURE_GRAPH_Y_MIN, TEMPERATURE_GRAPH_Y_MAX)
        axis.grid(which='major', color='0.85', linestyle='-', linewidth=0.8)

        if not self.node.temperature_samples:
            axis.text(0.5, 0.5, 'Waiting for temperature data', transform=axis.transAxes, ha='center', va='center')
            axis.set_xlim(0, 1)
            return

        sample_numbers = list(self.node.temperature_samples)
        temperatures = list(self.node.temperature_values)
        axis.plot(sample_numbers, temperatures, color='tab:red', linewidth=1.6)
        axis.set_xlim(max(1, sample_numbers[0]), sample_numbers[-1] + 1)

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()

    app = QApplication(sys.argv)
    window = MappingWindow(node)
    window.resize(1400, 1000)
    window.show()

    try:
        # Changed to app.exec_() for explicit Qt5 compatibility
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    main()
