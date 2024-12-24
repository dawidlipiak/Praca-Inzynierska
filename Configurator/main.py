import sys
import hid
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QTextEdit, QSlider, QHBoxLayout, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QTransform
from config_commands import *

class HIDControllerApp(QWidget):
    DEVICE_VID = 0x1209  # 4617 in decimal
    DEVICE_PID = 0xFFB0  # 65456 in decimal

    def __init__(self):
        super().__init__()
        self.device = None  # Variable to store device connection
        self.max_rotation = 900  # Default maximum rotation angle
        self.default_power = 6000  # Add default power value
        self.default_idle_spring = 128  # Add default idle spring value
        self.init_ui()
        self.show_connection_ui()

    def init_ui(self):
        """Creates the initial application interface."""
        self.setWindowTitle("Steering Wheel Controller")
        self.setGeometry(100, 100, 700, 800)
        # self.setFixedSize(700, 800)
        
        self.layout = QVBoxLayout(self)

        # --- Connection UI ---
        self.connection_widget = QWidget(self)
        connection_layout = QVBoxLayout(self.connection_widget)

        self.vid_label = QLabel("Enter VID (Decimal):")
        connection_layout.addWidget(self.vid_label)

        self.vid_input = QLineEdit()
        self.vid_input.setText(str(self.DEVICE_VID))
        connection_layout.addWidget(self.vid_input)

        self.pid_label = QLabel("Enter PID (Decimal):")
        connection_layout.addWidget(self.pid_label)

        self.pid_input = QLineEdit()
        self.pid_input.setText(str(self.DEVICE_PID))
        connection_layout.addWidget(self.pid_input)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)
        connection_layout.addWidget(self.connect_button)
        
        self.padding_widget = QWidget()
        self.padding_widget.setFixedHeight(500)
        connection_layout.addWidget(self.padding_widget)

        self.layout.addWidget(self.connection_widget)

        # --- Configurator UI ---
        self.configurator_widget = QWidget(self)
        self.configurator_widget.hide()  # Initially hide this interface
        configurator_layout = QVBoxLayout(self.configurator_widget)

        # Add Initialize and Set center position buttons in top right corner
        top_layout = QHBoxLayout()
        top_layout.addStretch()  # This pushes the buttons to the right
        
        self.initialize_button = QPushButton("Initialize")
        self.initialize_button.setFixedSize(100, 30)  # Set fixed size for the button
        
        self.set_center_button = QPushButton("Set center position")
        self.set_center_button.setFixedSize(150, 30)  # Set fixed width for the button
        
        top_layout.addWidget(self.set_center_button)
        top_layout.addWidget(self.initialize_button)
        configurator_layout.addLayout(top_layout)

        # Creating graphical view for the steering wheel
        self.graphics_view = QGraphicsView(self)
        self.scene = QGraphicsScene(self.graphics_view)
        self.graphics_view.setScene(self.scene)
        pixmap = QPixmap("img/steeringWheel.png")
        self.pixmap = pixmap.scaled(int(pixmap.width() * 0.3), int(pixmap.height() * 0.3), Qt.AspectRatioMode.KeepAspectRatio)
        self.wheel_item = QGraphicsPixmapItem(self.pixmap)
        self.wheel_item.setTransformOriginPoint(self.pixmap.width() / 2, self.pixmap.height() / 2)  # Set image center as rotation point
        self.scene.addItem(self.wheel_item)

        self.graphics_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        configurator_layout.addWidget(self.graphics_view)

        # Display current angle
        self.angle_label = QLabel("Angle: 0°", self)
        self.angle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        configurator_layout.addWidget(self.angle_label)

        # X-axis position slider
        x_slider_layout = QHBoxLayout()
        self.x_slider = QSlider(Qt.Orientation.Horizontal, self)
        self.x_slider.setMinimum(-32768)
        self.x_slider.setMaximum(32767)
        self.x_slider.setValue(0)
        self.x_slider.setEnabled(False)
        x_slider_layout.addWidget(self.x_slider)
        
        # X axis text
        self.max_rotation_display = QLabel(f"Current position")
        self.max_rotation_display.setFixedWidth(110)
        x_slider_layout.addWidget(self.max_rotation_display)
        
        configurator_layout.addLayout(x_slider_layout)

        # Max rotation angle and slider layout
        max_rotation_layout = QHBoxLayout()

        # Slider for max rotation
        self.max_rotation_slider = QSlider(Qt.Orientation.Horizontal)
        self.max_rotation_slider.setMinimum(0)
        self.max_rotation_slider.setMaximum(1440)
        self.max_rotation_slider.setValue(self.max_rotation)
        self.max_rotation_slider.valueChanged.connect(self.update_max_rotation_from_slider)
        max_rotation_layout.addWidget(self.max_rotation_slider)

        # Display current max rotation
        self.max_rotation_display = QLabel(f"Max angle")
        max_rotation_layout.addWidget(self.max_rotation_display)
    
        # Max rotation angle input
        self.max_rotation_input = QLineEdit()
        self.max_rotation_input.setFixedWidth(50)
        self.max_rotation_input.setPlaceholderText("Enter max rotation")
        self.max_rotation_input.setText(str(self.max_rotation))
        self.max_rotation_input.editingFinished.connect(self.update_max_rotation_from_input)
        max_rotation_layout.addWidget(self.max_rotation_input)

        configurator_layout.addLayout(max_rotation_layout)

        # Power control layout
        power_layout = QHBoxLayout()

        # Slider for power control
        self.power_slider = QSlider(Qt.Orientation.Horizontal)
        self.power_slider.setMinimum(0)
        self.power_slider.setMaximum(9000)
        self.power_slider.setValue(self.default_power)  # Set default value to 6000
        self.power_slider.valueChanged.connect(self.update_power_from_slider)
        power_layout.addWidget(self.power_slider)

        # Display current power
        self.power_display = QLabel(f"Power")
        self.power_display.setFixedWidth(55)
        power_layout.addWidget(self.power_display)
    
        # Power input field
        self.power_input = QLineEdit()
        self.power_input.setFixedWidth(50)
        self.power_input.setPlaceholderText("Enter power")
        self.power_input.setText(str(self.default_power))  # Set default value to 6000
        self.power_input.editingFinished.connect(self.update_power_from_input)
        power_layout.addWidget(self.power_input)

        configurator_layout.addLayout(power_layout)

        # Idle spring control layout
        idle_spring_layout = QHBoxLayout()

        # Slider for idle spring control
        self.idle_spring_slider = QSlider(Qt.Orientation.Horizontal)
        self.idle_spring_slider.setMinimum(0)
        self.idle_spring_slider.setMaximum(255)
        self.idle_spring_slider.setValue(self.default_idle_spring)
        self.idle_spring_slider.valueChanged.connect(self.update_idle_spring_from_slider)
        idle_spring_layout.addWidget(self.idle_spring_slider)

        # Display current idle spring
        self.idle_spring_display = QLabel(f"Idle spring")
        idle_spring_layout.addWidget(self.idle_spring_display)
    
        # Idle spring input field
        self.idle_spring_input = QLineEdit()
        self.idle_spring_input.setFixedWidth(50)
        self.idle_spring_input.setPlaceholderText("Enter value")
        self.idle_spring_input.setText(str(self.default_idle_spring))
        self.idle_spring_input.editingFinished.connect(self.update_idle_spring_from_input)
        idle_spring_layout.addWidget(self.idle_spring_input)

        configurator_layout.addLayout(idle_spring_layout)

        # Display device feedback
        self.feedback_display = QTextEdit()
        self.feedback_display.setReadOnly(True)
        self.feedback_display.setFixedHeight(150)  # Set fixed height to 150
        self.feedback_display.setPlaceholderText("Device feedback will appear here...")
        configurator_layout.addWidget(self.feedback_display)

        self.layout.addWidget(self.configurator_widget)

        # Timer for receiving HID data
        self.feedback_timer = QTimer()
        self.feedback_timer.timeout.connect(self.receive_feedback)

        # Connect button signals
        self.initialize_button.clicked.connect(self.initialize_device)
        self.set_center_button.clicked.connect(self.set_center_position)

    def show_connection_ui(self):
        """Shows the connection interface."""
        self.connection_widget.show()
        self.configurator_widget.hide()

    def show_configurator_ui(self):
        """Shows the configurator interface."""
        self.connection_widget.hide()
        self.configurator_widget.show()

    def connect_device(self):
        """Connects to HID device and switches to main interface."""
        try:
            vid = int(self.vid_input.text())
            pid = int(self.pid_input.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid decimal values for VID and PID.")
            return

        try:
            self.device = hid.device()
            self.device.open(vid, pid)
            QMessageBox.information(self, "Connected", "Connected to HID device!")
            self.show_configurator_ui()
            self.feedback_timer.start(50)  # Start timer for HID reading
        except Exception as e:
            QMessageBox.critical(self, "Connection Failed", f"Connection failed: {e}")
            self.device = None

    def receive_feedback(self):
        """Receives HID report and updates interface."""
        if not self.device:
            return

        try:
            # Read report - max size is 21 bytes (1 for report ID, 20 for data in case of response command)
            report = self.device.read(21)
            if not report:
                return
            print(report)
            # Handle different report types based on report ID
            report_id = report[0]

            if report_id == REPORT_ID_WHEEL:  # Wheel position report
                if len(report) >= 3:  # 1 byte report ID + 2 bytes axis position
                    x_value = int.from_bytes(report[1:3], byteorder="little", signed=True)
                    self.x_slider.setValue(x_value)

                    # Calculate angle
                    angle = (x_value / 32767) * (self.max_rotation / 2)
                    self.angle_label.setText(f"Angle: {angle:.1f}°")
                    
                    # Rotate steering wheel image
                    self.wheel_item.setRotation(angle)

            elif report_id == REPORT_ID_RESPONSE:  # Command response report
                if len(report) >= 2:  # At least report ID and status
                    response_text = ""
                    # Convert remaining bytes to ASCII, stopping at null terminator
                    for b in report[1:]:
                        if b == 0:
                            break
                        response_text += chr(b)
                    
                    if response_text:
                        self.feedback_display.append(f"Device response: {response_text}")

        except Exception as e:
            self.feedback_display.append(f"Error reading feedback: {e}")
            self.feedback_timer.stop()

    def update_max_rotation(self):
        """Updates the maximum rotation angle."""
        self.max_rotation = self.max_rotation_slider.value()
        self.max_rotation_label.setText(f"{self.max_rotation}°")
    
    def update_max_rotation_from_input(self):
        """Update max rotation from the input field."""
        try:
            new_rotation = int(self.max_rotation_input.text())
            if 0 <= new_rotation <= 1440:
                self.max_rotation = new_rotation
                self.max_rotation_slider.setValue(self.max_rotation)
            else:
                QMessageBox.warning(self, "Invalid Value", "Enter a value between 0 and 1440.")
                self.max_rotation_input.setText(str(self.max_rotation))
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Enter a numeric value.")
            self.max_rotation_input.setText(str(self.max_rotation))

    def update_max_rotation_from_slider(self):
        """Update max rotation from the slider."""
        self.max_rotation = self.max_rotation_slider.value()
        self.max_rotation_input.setText(str(self.max_rotation))
        self.send_max_rotation_to_device(self.max_rotation)

    def send_max_rotation_to_device(self, max_rotation):
        """Send max rotation value to device."""
        if self.device:
            try:
                # Convert max_rotation to bytes (little-endian)
                max_rotation_bytes = max_rotation.to_bytes(2, byteorder='little')
                self.send_command(CMD_SET_MAX_ANGLE, max_rotation_bytes)
                self.feedback_display.append("Sending set max angle command successful")
            except Exception as e:
                self.feedback_display.append("Sending set max angle command failed")

    def update_power_from_input(self):
        """Update power from the input field."""
        try:
            new_power = int(self.power_input.text())
            if 0 <= new_power <= 9000:
                self.power_slider.setValue(new_power)
                self.send_power_to_device(new_power)
            else:
                QMessageBox.warning(self, "Invalid Value", "Enter a value between 0 and 9000.")
                self.power_input.setText(str(self.power_slider.value()))
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Enter a numeric value.")
            self.power_input.setText(str(self.power_slider.value()))

    def update_power_from_slider(self):
        """Update power from the slider."""
        power = self.power_slider.value()
        self.power_input.setText(str(power))
        self.send_power_to_device(power)

    def send_power_to_device(self, power):
        """Send power value to device."""
        if self.device:
            try:
                # Convert to 4 bytes
                power_bytes = power.to_bytes(2, byteorder='little')
                self.send_command(CMD_SET_POWER, power_bytes)
                self.feedback_display.append("Sending set power command successful")
            except Exception as e:
                self.feedback_display.append("Sending set power command failed")

    def update_idle_spring_from_input(self):
        """Update idle spring from the input field."""
        try:
            new_idle_spring = int(self.idle_spring_input.text())
            if 0 <= new_idle_spring <= 255:
                self.idle_spring_slider.setValue(new_idle_spring)
                self.send_idle_spring_to_device(new_idle_spring)
            else:
                QMessageBox.warning(self, "Invalid Value", "Enter a value between 0 and 255.")
                self.idle_spring_input.setText(str(self.idle_spring_slider.value()))
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Enter a numeric value.")
            self.idle_spring_input.setText(str(self.idle_spring_slider.value()))

    def update_idle_spring_from_slider(self):
        """Update idle spring from the slider."""
        idle_spring = self.idle_spring_slider.value()
        self.idle_spring_input.setText(str(idle_spring))
        self.send_idle_spring_to_device(idle_spring)

    def send_idle_spring_to_device(self, idle_spring):
        """Send idle spring value to device."""
        if self.device:
            if self.send_command(CMD_SET_SPRING, idle_spring):
                self.feedback_display.append("Sending set spring command successful")
            else:
                self.feedback_display.append("Sending set spring command failed")

    def closeEvent(self, event):
        """Closes HID connection when closing the application."""
        if self.device:
            self.device.close()
        event.accept()

    def send_command(self, cmd, data=None):
        """Send a command to the device and wait for response."""
        if not self.device:
            self.feedback_display.append("Error: Device not connected")
            return False

        try:
            report = [REPORT_ID_CONFIG, cmd]  # Report ID (0x14), command
            if data:
                # Add the length of the data array as the next byte in the report
                report.append(len(data))  # Data length
                
                # Add all bytes from the data array to the end of the report
                # This contains the actual payload/parameters for the command
                report.extend(data)
            else:
                report.append(0)  # No data
                
            # Pad to 4 bytes
            report.extend([0] * (4 - len(report)))
            
            result = self.device.write(report)
            print("Send ", result, " ", report)
            return True  # Return True if write was successful
        except Exception as e:
            self.feedback_display.append(f"Error sending command: {e}")
            return False

    def initialize_device(self):
        """Initialize the device."""
        if self.send_command(CMD_INITIALIZE):
            self.feedback_display.append("Sending initialize command successful")
        else:
            self.feedback_display.append("Sending initialize command failed")

    def set_center_position(self):
        """Set the center position."""
        if self.send_command(CMD_SET_CENTER):
            self.feedback_display.append("Sending set center command successful")
        else:
            self.feedback_display.append("Sending set center command failed")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HIDControllerApp()
    window.show()
    sys.exit(app.exec())
