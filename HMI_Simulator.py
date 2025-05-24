# HMI Simulator with PID Temperature Control, Two Tanks and a Valve
# ---------------------------------------------------------------
# Run: python hmi_simulator.py
# Dependencies: PyQt5  (pip install pyqt5)
#
# This program simulates a simple process consisting of two liquid tanks connected
# by a valve and a heater controlled by a PID controller. The GUI is drawn with
# PyQt5 and updates every 100 ms. Tank 1 drains through the valve into Tank 2.
# A heater in Tank 2 is modulated by a PID loop to hold the requested temperature
# set‑point.                                                  

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
        self.valve_open = False
        self.flow_rate_pct_per_s = 5.0  # %/s when valve open

        # PID to control heater power (0‑100)
        self.pid = PIDController(kp=2.0, ki=0.5, kd=0.1, setpoint=60.0)

        # ----- UI widgets ----------------------------------------------
        self.tank1 = TankWidget("Tank 1")
        self.tank2 = TankWidget("Tank 2")

        self.valve_btn = QPushButton("Valve: CLOSED")
        self.valve_btn.setCheckable(True)
        self.valve_btn.clicked.connect(self._toggle_valve)

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
        grid.addWidget(self.valve_btn, 0, 1, alignment=Qt.AlignCenter)
        grid.addWidget(self.tank2, 0, 2)
        grid.addWidget(self.setpoint_slider, 0, 3, alignment=Qt.AlignHCenter)
        grid.addWidget(self.lbl_setpoint, 1, 3, alignment=Qt.AlignHCenter)
        grid.addWidget(self.lbl_temp, 1, 2, alignment=Qt.AlignHCenter)

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
    def _toggle_valve(self, checked: bool):
        self.valve_open = checked
        self.valve_btn.setText("Valve: OPEN" if checked else "Valve: CLOSED")

    def _set_setpoint(self, value: int):
        self.pid.setpoint = float(value)
        self.lbl_setpoint.setText(f"Set‑point:\n{value:.0f} °C")

    # ---------------------------------------------------------------------
    #                          Simulation step
    # ---------------------------------------------------------------------
    def _update_sim(self):
        dt = self.DT

        # Flow between tanks
        if self.valve_open and self.level1 > 0.0:
            dlevel = min(self.flow_rate_pct_per_s * dt, self.level1)
            self.level1 -= dlevel
            self.level2 = min(self.level2 + dlevel, 100.0)

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


