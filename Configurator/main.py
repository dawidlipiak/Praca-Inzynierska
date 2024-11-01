import sys
import hid
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, QMessageBox, QTextEdit
from PyQt6.QtCore import QTimer

class HIDControllerApp(QWidget):
    def __init__(self):
        super().__init__()
        self.device = None  # Zmienna do przechowywania połączenia z urządzeniem
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("HID Controller")
        layout = QVBoxLayout()

        # VID Label and Input
        self.vid_label = QLabel("Enter VID (Decimal):")
        layout.addWidget(self.vid_label)
        
        self.vid_input = QLineEdit()
        self.vid_input.setPlaceholderText("e.g. 1155")
        layout.addWidget(self.vid_input)

        # PID Label and Input
        self.pid_label = QLabel("Enter PID (Decimal):")
        layout.addWidget(self.pid_label)

        self.pid_input = QLineEdit()
        self.pid_input.setPlaceholderText("e.g. 22352")
        layout.addWidget(self.pid_input)

        # Connect Button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)
        layout.addWidget(self.connect_button)

        # Left Button
        self.left_button = QPushButton("Obróć w lewo")
        self.left_button.clicked.connect(self.rotate_left)
        self.left_button.setEnabled(False)  # Disable until connected
        layout.addWidget(self.left_button)

        # Right Button
        self.right_button = QPushButton("Obróć w prawo")
        self.right_button.clicked.connect(self.rotate_right)
        self.right_button.setEnabled(False)  # Disable until connected
        layout.addWidget(self.right_button)
        
        # Output Display for Feedback
        self.feedback_display = QTextEdit()
        self.feedback_display.setReadOnly(True)
        self.feedback_display.setPlaceholderText("Device feedback will appear here...")
        layout.addWidget(self.feedback_display)

        self.setLayout(layout)
        
        # Timer to regularly check for feedback
        self.feedback_timer = QTimer()
        self.feedback_timer.timeout.connect(self.receive_feedback)

    def connect_device(self):
        # Pobierz VID i PID z pól wejściowych
        try:
            vid = int(self.vid_input.text())
            pid = int(self.pid_input.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter valid decimal values for VID and PID.")
            return

        # Próbujemy połączyć się z urządzeniem
        try:
            self.device = hid.device()
            self.device.open(vid, pid)
            QMessageBox.information(self, "Connected", "Połączono z urządzeniem HID!")
            
            # Aktywuj przyciski sterujące
            self.left_button.setEnabled(True)
            self.right_button.setEnabled(True)
        except Exception as e:
            QMessageBox.critical(self, "Connection Failed", f"Nie udało się połączyć: {e}")
            self.device = None

    def rotate_left(self):
        if self.device:
            try:
                # Wysyłamy raport dla obrotu w lewo
                report = [0x00, 0x01] + [0x00] * 62  # 8-bajtowy raport z komendą "lewo"
                write_info = self.device.write(report)
                print("Write attempt info: ", write_info)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to send report: {e}")

    def rotate_right(self):
        if self.device:
            try:
                # Wysyłamy raport dla obrotu w prawo
                report = [0x00, 0x02] + [0x00] * 62  # 8-bajtowy raport z komendą "prawo"
                write_info = self.device.write(report)
                print("Write attempt info: ", write_info)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to send report: {e}")

    def receive_feedback(self):
        if self.device:
            try:
                # Odczytanie zwrotnego raportu HID
                report = self.device.read(64)  # Odczytaj raport do 64 bajtów
                if report:
                    # Wyświetl raport w polu tekstowym
                    feedback = f"Feedback: {report[0]}"
                    self.feedback_display.append(feedback)
            except Exception as e:
                self.feedback_display.append(f"Error reading feedback: {e}")
                self.feedback_timer.stop()

    def closeEvent(self, event):
        # Zamykamy połączenie przy zamykaniu aplikacji
        if self.device:
            self.device.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = HIDControllerApp()
    window.show()
    sys.exit(app.exec())
