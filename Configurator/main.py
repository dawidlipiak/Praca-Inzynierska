import sys
import hid
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QTextEdit, QSlider, QHBoxLayout, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QTransform

class HIDControllerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.device = None  # Zmienna do przechowywania połączenia z urządzeniem
        self.max_rotation = 900  # Domyślny maksymalny kąt obrotu
        self.init_ui()
        self.show_connection_ui()

    def init_ui(self):
        """Tworzy początkowy interfejs aplikacji."""
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
        self.vid_input.setPlaceholderText("e.g. 1155")
        connection_layout.addWidget(self.vid_input)

        self.pid_label = QLabel("Enter PID (Decimal):")
        connection_layout.addWidget(self.pid_label)

        self.pid_input = QLineEdit()
        self.pid_input.setPlaceholderText("e.g. 22352")
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
        self.configurator_widget.hide()  # Na początku ukrywamy ten interfejs
        configurator_layout = QVBoxLayout(self.configurator_widget)

        # Tworzenie widoku graficznego dla kierownicy
        self.graphics_view = QGraphicsView(self)
        self.scene = QGraphicsScene(self.graphics_view)
        self.graphics_view.setScene(self.scene)
        pixmap = QPixmap("img/steeringWheel.png")
        self.pixmap = pixmap.scaled(int(pixmap.width() * 0.3), int(pixmap.height() * 0.3), Qt.AspectRatioMode.KeepAspectRatio)
        self.wheel_item = QGraphicsPixmapItem(self.pixmap)
        self.wheel_item.setTransformOriginPoint(self.pixmap.width() / 2, self.pixmap.height() / 2)  # Ustawienie środka obrazu jako punktu obrotu
        self.scene.addItem(self.wheel_item)
        # self.wheel_item.setPos(
        #     (self.graphics_view.viewport().width() - self.pixmap.width()) / 2,
        #     (self.graphics_view.viewport().height() - self.pixmap.height()) / 2
        # )

        self.graphics_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        configurator_layout.addWidget(self.graphics_view)


        # Wyświetlanie aktualnego kąta
        self.angle_label = QLabel("Angle: 0°", self)
        self.angle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        configurator_layout.addWidget(self.angle_label)

        # Suwak pozycji osi X
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
        
        # Add the axis layout to the configurator layout
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
    
        # Max rotation angle input (editable field)
        self.max_rotation_input = QLineEdit()
        self.max_rotation_input.setFixedWidth(50)  # Optional: adjust width
        self.max_rotation_input.setPlaceholderText("Enter max rotation")
        self.max_rotation_input.setText(str(self.max_rotation))  # Default value
        self.max_rotation_input.editingFinished.connect(self.update_max_rotation_from_input)
        max_rotation_layout.addWidget(self.max_rotation_input)
        

        # Add the layout to the configurator layout
        configurator_layout.addLayout(max_rotation_layout)


        # Wyświetlanie feedbacku z urządzenia
        self.feedback_display = QTextEdit()
        self.feedback_display.setReadOnly(True)
        self.feedback_display.setPlaceholderText("Device feedback will appear here...")
        configurator_layout.addWidget(self.feedback_display)

        self.layout.addWidget(self.configurator_widget)

        # Timer do odbierania danych HID
        self.feedback_timer = QTimer()
        self.feedback_timer.timeout.connect(self.receive_feedback)

    def show_connection_ui(self):
        """Pokazuje interfejs połączenia."""
        self.connection_widget.show()
        self.configurator_widget.hide()

    def show_configurator_ui(self):
        """Pokazuje interfejs konfiguratora."""
        self.connection_widget.hide()
        self.configurator_widget.show()

    def connect_device(self):
        """Łączy z urządzeniem HID i przełącza na główny interfejs."""
        try:
            vid = int(self.vid_input.text())
            pid = int(self.pid_input.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid decimal values for VID and PID.")
            return

        try:
            self.device = hid.device()
            self.device.open(vid, pid)
            QMessageBox.information(self, "Connected", "Połączono z urządzeniem HID!")
            self.show_configurator_ui()
            self.feedback_timer.start(50)  # Start timer do odczytu HID
        except Exception as e:
            QMessageBox.critical(self, "Connection Failed", f"Nie udało się połączyć: {e}")
            self.device = None

    def receive_feedback(self):
        """Odbiera raport HID i aktualizuje interfejs."""
        if not self.device:
            return

        try:
            report = self.device.read(16)  # Timeout w milisekundach
            if report:
                # Przetwarzanie raportu
                x_value = int.from_bytes(report[1:3], byteorder="little", signed=True)
                self.x_slider.setValue(x_value)

                # Przeliczanie kąta
                angle = (x_value / 32767) * (self.max_rotation / 2)
                self.angle_label.setText(f"Angle: {angle:.1f}°")
                
                # Obracanie obrazu kierownicy
                self.wheel_item.setRotation(angle)

        except Exception as e:
            self.feedback_display.append(f"Error reading feedback: {e}")
            self.feedback_timer.stop()

    def update_max_rotation(self):
        """Aktualizuje maksymalny kąt obrotu."""
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


    def closeEvent(self, event):
        """Zamyka połączenie HID przy zamykaniu aplikacji."""
        if self.device:
            self.device.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HIDControllerApp()
    window.show()
    sys.exit(app.exec())
