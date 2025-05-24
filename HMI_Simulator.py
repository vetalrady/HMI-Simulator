# HMI Simulator with PID Temperature Control and Simple Fluid Network
# ------------------------------------------------------------------
# Run: python hmi_simulator.py
# Dependencies: PyQt5  (pip install pyqt5)
#
# This program simulates a simple process consisting of two liquid tanks
# connected by a valve and a heater controlled by a PID controller.  The GUI is
# drawn with PyQt5 and updates every 100 ms.  The original example has been
# extended with a small fluid network consisting of three valves:
#   * "Fill"   – adds fluid to Tank 1
#   * "Valve"  – connects Tank 1 to Tank 2 (flow depends on level difference)
#   * "Drain"  – removes fluid from Tank 2
# A heater in Tank 2 is modulated by a PID loop to hold the requested
# temperature set‑point.

import sys
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLabel,
    QSlider,
    QPushButton,
    QGridLayout,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen


class PIDController:
    """A simple PID controller."""

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, pv: float, dt: float) -> float:
        """Return control output for the given process value (pv)."""
        error = self.setpoint - pv
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class TankWidget(QWidget):
    """Canvas that draws a vertical tank with fill level and temperature."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.level = 0.0      # percent (0‑100)
        self.temperature = 20.0  # °C (for colour only)
        self.setMinimumSize(120, 300)

    # ----- Public API -----------------------------------------------------
    def set_level(self, percent: float):
        self.level = max(0.0, min(percent, 100.0))
        self.update()

    def set_temperature(self, temp_c: float):
        self.temperature = temp_c
        self.update()

    # ----- Painting -------------------------------------------------------
    def paintEvent(self, evt):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw tank outline
        margin = 20
        tank_rect = self.rect().adjusted(margin, margin, -margin, -60)
        painter.setPen(QPen(Qt.black, 2))
        painter.drawRect(tank_rect)

        # Compute fluid rectangle based on level
        fill_h = tank_rect.height() * self.level / 100.0
        fluid_rect = tank_rect.adjusted(0, tank_rect.height() - fill_h, 0, 0)

        # Colour ranges from blue (cold) to red (hot)
        t_ratio = max(0.0, min((self.temperature - 20.0) / 80.0, 1.0))
        color = QColor(int(0 + t_ratio * 255), int(100 - t_ratio * 100), int(255 - t_ratio * 255))
        painter.fillRect(fluid_rect, color)

        # Draw title and numeric values
        painter.drawText(
            self.rect().adjusted(0, 0, 0, -30),
            Qt.AlignHCenter | Qt.AlignTop,
            f"{self.title}\nLevel: {self.level:4.0f}%",
        )
        painter.drawText(
            self.rect().adjusted(0, 0, 0, -10),
            Qt.AlignHCenter | Qt.AlignBottom,
            f"{self.temperature:4.1f} °C",
        )


class PipeWidget(QWidget):
    """Simple horizontal pipe widget with basic flow animation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.flow = 0.0
        self._phase = 0.0
        self.setMinimumSize(120, 40)

    def set_flow(self, flow: float):
        """Set current flow and trigger repaint."""
        self.flow = flow
        # phase advances proportionally to flow magnitude
        self._phase = (self._phase + abs(flow)) % 10.0
        self.update()

    def paintEvent(self, evt):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(10, 10, -10, -10)
        painter.setPen(QPen(Qt.black, 2))
        painter.drawRect(rect)

        if self.flow > 0.001:
            painter.fillRect(rect.adjusted(1, 1, -1, -1), QColor(160, 200, 255))
            step = 10
            offset = int(self._phase) % step
            for x in range(rect.left() + offset, rect.right(), step):
                painter.drawLine(x, rect.top(), x - 6, rect.bottom())
        else:
            painter.fillRect(rect.adjusted(1, 1, -1, -1), QColor(230, 230, 230))


class MainWindow(QMainWindow):
    """Main application window holding all widgets and the simulation loop."""

    DT = 0.1  # simulation timestep in seconds (100 ms)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Industrial HMI Simulator")

        # ----- Process states -------------------------------------------
        self.level1 = 80.0  # % full
        self.level2 = 20.0  # % full
        self.temp2 = 20.0   # °C

        # Valve states -------------------------------------------------
        self.valve_fill_open = False    # feed into Tank 1
        self.valve12_open = False       # between Tank 1 and 2
        self.valve_drain_open = False   # drain from Tank 2

        # Flow coefficients (approx %/s) --------------------------------
        self.fill_rate = 5.0
        self.drain_rate = 5.0
        self.pipe_coeff = 3.0  # affects flow between tanks

        # PID to control heater power (0‑100)
        self.pid = PIDController(kp=2.0, ki=0.5, kd=0.1, setpoint=60.0)

        # ----- UI widgets ----------------------------------------------
        self.tank1 = TankWidget("Tank 1")
        self.tank2 = TankWidget("Tank 2")
        self.pipe = PipeWidget()

        self.valve_btn = QPushButton("Valve: CLOSED")  # between tanks
        self.valve_btn.setCheckable(True)
        self.valve_btn.clicked.connect(self._toggle_valve12)

        self.fill_btn = QPushButton("Fill: CLOSED")
        self.fill_btn.setCheckable(True)
        self.fill_btn.clicked.connect(self._toggle_fill)

        self.drain_btn = QPushButton("Drain: CLOSED")
        self.drain_btn.setCheckable(True)
        self.drain_btn.clicked.connect(self._toggle_drain)

        self.setpoint_slider = QSlider(Qt.Vertical)
        self.setpoint_slider.setRange(20, 100)
        self.setpoint_slider.setValue(int(self.pid.setpoint))
        self.setpoint_slider.valueChanged.connect(self._set_setpoint)

        self.lbl_setpoint = QLabel(f"Set‑point:\n{self.pid.setpoint:.0f} °C")
        self.lbl_temp = QLabel(f"T₂: {self.temp2:.1f} °C")
        self.lbl_temp.setAlignment(Qt.AlignCenter)

        # Layout ---------------------------------------------------------
        grid = QGridLayout()
        grid.addWidget(self.tank1, 0, 0)
        grid.addWidget(self.pipe, 0, 1, alignment=Qt.AlignCenter)
        grid.addWidget(self.tank2, 0, 2)
        grid.addWidget(self.setpoint_slider, 0, 3, alignment=Qt.AlignHCenter)

        grid.addWidget(self.fill_btn, 1, 0, alignment=Qt.AlignCenter)
        grid.addWidget(self.valve_btn, 1, 1, alignment=Qt.AlignCenter)
        grid.addWidget(self.drain_btn, 1, 2, alignment=Qt.AlignCenter)
        grid.addWidget(self.lbl_setpoint, 1, 3, alignment=Qt.AlignHCenter)
        grid.addWidget(self.lbl_temp, 2, 2, alignment=Qt.AlignHCenter)

        central = QWidget()
        central.setLayout(grid)
        self.setCentralWidget(central)

        # ----- Timer ----------------------------------------------------
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_sim)
        self._timer.start(int(self.DT * 1000))

    # ---------------------------------------------------------------------
    #                             Call‑backs
    # ---------------------------------------------------------------------
    def _toggle_valve12(self, checked: bool):
        self.valve12_open = checked
        self.valve_btn.setText("Valve: OPEN" if checked else "Valve: CLOSED")

    def _toggle_fill(self, checked: bool):
        self.valve_fill_open = checked
        self.fill_btn.setText("Fill: OPEN" if checked else "Fill: CLOSED")

    def _toggle_drain(self, checked: bool):
        self.valve_drain_open = checked
        self.drain_btn.setText("Drain: OPEN" if checked else "Drain: CLOSED")

    def _set_setpoint(self, value: int):
        self.pid.setpoint = float(value)
        self.lbl_setpoint.setText(f"Set‑point:\n{value:.0f} °C")

    # ---------------------------------------------------------------------
    #                          Simulation step
    # ---------------------------------------------------------------------
    def _update_sim(self):
        dt = self.DT

        flow12 = 0.0

        # Fill Tank 1 from feed
        if self.valve_fill_open:
            self.level1 = min(self.level1 + self.fill_rate * dt, 100.0)

        # Flow between tanks (simple sqrt(head) relation)
        if self.valve12_open and self.level1 > self.level2:
            head = self.level1 - self.level2
            flow = self.pipe_coeff * (head ** 0.5) * dt
            flow = min(flow, self.level1)
            self.level1 -= flow
            old_level2 = self.level2
            self.level2 = min(self.level2 + flow, 100.0)
            flow12 = flow
            # mix incoming ambient fluid (20°C) with tank2 contents
            if self.level2 > 0:
                self.temp2 = (
                    self.temp2 * old_level2 + 20.0 * flow
                ) / self.level2

        # Drain Tank 2
        if self.valve_drain_open:
            self.level2 = max(self.level2 - self.drain_rate * dt, 0.0)

        # Heater power from PID (0‑100)
        heater_power = max(0.0, min(100.0, self.pid.update(self.temp2, dt)))

        # Simple thermal model: add heat from heater, lose heat to ambient (20 °C)
        heat_gain = heater_power * 0.2 * dt   # 0.2 °C per %power‑second
        heat_loss = 0.01 * (self.temp2 - 20.0) * dt  # proportional cooling
        self.temp2 += heat_gain - heat_loss

        # Update widgets
        self.tank1.set_level(self.level1)
        self.tank2.set_level(self.level2)
        self.tank2.set_temperature(self.temp2)
        self.lbl_temp.setText(f"T₂: {self.temp2:.1f} °C")
        self.pipe.set_flow(flow12 / dt)


# -------------------------------------------------------------------------
#                                 Main
# -------------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


