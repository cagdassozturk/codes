import sys
from PyQt6 import QtCore, QtGui, QtWidgets
import serial
import pyqtgraph as pg

#ser = serial.Serial('COM3', baudrate=9600, timeout=1)


def send_command(button_number):
    # İlk byte buton numarasına eşit olmalıdır.
    first_byte = button_number

    # İkinci byte, ilk byte'ın bitlerinin tersi olmalıdır.
    second_byte = ~button_number & 0xFF

    # 2 byte'lık paketi gönder
    #ser.write(bytes([first_byte, second_byte]))

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setGeometry(30, 300, 400, 550)
        self.setWindowTitle("Real-Time Data Plotting")
        self.graphWidget = pg.PlotWidget()
        self.setCentralWidget(self.graphWidget)
        self.graphWidget.setBackground('w')
        self.graphWidget.showGrid(x=True, y=True)
        self.graphWidget.setLabel('left', 'Values')
        self.graphWidget.setLabel('bottom', 'Time')
        self.graphWidget.setTitle('Real-Time Data Plotting')



        self.latitude = []
        self.longitude = []
        self.speed = []
        self.acceleration = []
        self.temperature = []
        self.time = []

        self.pen = pg.mkPen(color=(255, 0, 0))
        self.plotLatitude = self.graphWidget.plot(pen=self.pen)
        self.pen = pg.mkPen(color=(0, 255, 0))
        self.plotLongitude = self.graphWidget.plot(pen=self.pen)
        self.pen = pg.mkPen(color=(0, 0, 255))
        self.plotSpeed = self.graphWidget.plot(pen=self.pen)
        self.pen = pg.mkPen(color=(255, 255, 0))
        self.plotAcceleration = self.graphWidget.plot(pen=self.pen)
        self.pen = pg.mkPen(color=(255, 0, 255))
        self.plotTemperature = self.graphWidget.plot(pen=self.pen)

    def update_data(self, values):
        self.latitude.append(float(values[0]))
        self.longitude.append(float(values[1]))
        self.speed.append(float(values[2]))
        self.acceleration.append(float(values[3]))
        self.temperature.append(float(values[4]))
        self.time.append(float(len(self.time) + 1))

        self.plotLatitude.setData(self.time, self.latitude)
        self.plotLongitude.setData(self.time, self.longitude)
        self.plotSpeed.setData(self.time, self.speed)
        self.plotAcceleration.setData(self.time, self.acceleration)
        self.plotTemperature.setData(self.time, self.temperature)

    def closeEvent(self, event):
        sys.exit()

class DataThread(QtCore.QThread):
    def __init__(self, serial, window):
        super().__init__()
        self.serial = serial
        self.window = window
        self.running = True

    def run(self):
        while self.running:
            try:
                # Read line from serial port
                line = self.serial.readline().decode().strip()

                # Split line into values
                values = line.split(",")

                # Update data
                self.window.update_data(values)
            except:
                pass

    def stop(self):
        self.running = False
        self.wait()

class SerialPort(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        # Create widgets
        self.baudrate_label = QtWidgets.QLabel("Baudrate:")
        self.baudrate_combo = QtWidgets.QComboBox()
        self.baudrate_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)

        # Create layout
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.baudrate_label)
        hbox.addWidget(self.baudrate_combo)
        hbox.addWidget(self.connect_button)
        hbox.addWidget(self.disconnect_button)

        # Create plot widgets
        self.latitude_plot = pg.PlotWidget()
        self.longitude_plot = pg.PlotWidget()
        self.speed_plot = pg.PlotWidget()
        self.acceleration_plot = pg.PlotWidget()
        self.temperature_plot = pg.PlotWidget()

        # Create layout for plot widgets
        plot_layout = QtWidgets.QGridLayout()
        plot_layout.addWidget(self.latitude_plot, 0, 0)
        plot_layout.addWidget(self.longitude_plot, 0, 1)
        plot_layout.addWidget(self.speed_plot, 1, 0)
        plot_layout.addWidget(self.acceleration_plot, 1, 1)
        plot_layout.addWidget(self.temperature_plot, 2, 0, 1, 2)

        # Set layout for main window
        vbox = QtWidgets.QVBoxLayout()
        vbox.addLayout(hbox)
        vbox.addLayout(plot_layout)
        self.setLayout(vbox)

        # Connect signals and slots
        self.connect_button.clicked.connect(self.connect_serial)
        self.disconnect_button.clicked.connect(self.disconnect_serial)

    def connect_serial(self):
        # Get selected baud rate
        baudrate = int(self.baudrate_combo.currentText())

        try:
            # Connect to serial port
            self.serial = serial.Serial('/dev/ttyACM0', baudrate)

            # Disable connect button and enable disconnect button
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)

            # Create data thread and start it
            self.data_thread = DataThread(self.serial, self)
            self.data_thread.start()
        except serial.SerialException as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e))

    def disconnect_serial(self):
        # Stop data thread
        self.data_thread.stop()

        # Disconnect from serial port
        self.serial.close()

        # Enable connect button and disable disconnect button
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)

    def update_data(self, values):
        # Update latitude plot
        self.latitude_plot.plot(values[0])

        # Update longitude plot
        self.longitude_plot.plot(values[1])

        # Update speed plot
        self.speed_plot.plot(values[2])

        # Update acceleration plot
        self.acceleration_plot.plot(values[3])

        # Update temperature plot
        self.temperature_plot.plot(values[4])

class SensorDataPlot(QtWidgets.QWidget):
    def init(self):
        super().init()
        self.init_ui()

    def init_ui(self):
        # Create latitude plot
        self.latitude_plot = pg.PlotWidget(title="Latitude")
        self.latitude_plot.setLabel("left", "Latitude", units="degrees")
        self.latitude_plot.showGrid(x=True, y=True)

        # Create longitude plot
        self.longitude_plot = pg.PlotWidget(title="Longitude")
        self.longitude_plot.setLabel("left", "Longitude", units="degrees")
        self.longitude_plot.showGrid(x=True, y=True)

        # Create speed plot
        self.speed_plot = pg.PlotWidget(title="Speed")
        self.speed_plot.setLabel("left", "Speed", units="m/s")
        self.speed_plot.showGrid(x=True, y=True)

        # Create acceleration plot
        self.acceleration_plot = pg.PlotWidget(title="Acceleration")
        self.acceleration_plot.setLabel("left", "Acceleration", units="m/s^2")
        self.acceleration_plot.showGrid(x=True, y=True)

        # Create temperature plot
        self.temperature_plot = pg.PlotWidget(title="Temperature")
        self.temperature_plot.setLabel("left", "Temperature", units="°C")
        self.temperature_plot.showGrid(x=True, y=True)

        # Create layout
        grid = QtWidgets.QGridLayout()
        grid.addWidget(self.latitude_plot, 0, 0)
        grid.addWidget(self.longitude_plot, 0, 1)
        grid.addWidget(self.speed_plot, 1, 0)
        grid.addWidget(self.acceleration_plot, 1, 1)
        grid.addWidget(self.temperature_plot, 2, 0, 1, 2)
        self.setLayout(grid)

    def update_data(self, values):
        # Update latitude plot
        self.latitude_plot.plot(values[0])

        # Update longitude plot
        self.longitude_plot.plot(values[1])

        # Update speed plot
        self.speed_plot.plot(values[2])

        # Update acceleration plot
        self.acceleration_plot.plot(values[3])

        # Update temperature plot
        self.temperature_plot.plot(values[4])

class DataWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
    def init_ui(self):
        # Create plots

        self.latitude_plot = PlotWidget("Latitude")
        self.longitude_plot = PlotWidget("Longitude")
        self.speed_plot = PlotWidget("Speed")
        self.acceleration_plot = PlotWidget("Acceleration")
        self.temperature_plot = PlotWidget("Temperature")

        # Create layout
        grid = QtWidgets.QGridLayout()
        grid.addWidget(self.latitude_plot, 0, 0)
        grid.addWidget(self.longitude_plot, 0, 1)
        grid.addWidget(self.speed_plot, 1, 0)
        grid.addWidget(self.acceleration_plot, 1, 1)
        grid.addWidget(self.temperature_plot, 2, 0, 1, 2)

        # Set layout
        self.setLayout(grid)
    def update_data(self, values):
        # Update latitude plot
        self.latitude_plot.plot(values[0])

        # Update longitude plot
        self.longitude_plot.plot(values[1])

        # Update speed plot
        self.speed_plot.plot(values[2])

        # Update acceleration plot
        self.acceleration_plot.plot(values[3])

        # Update temperature plot
        self.temperature_plot.plot(values[4])

class SecondWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        # Eve Dön Butonu
        btn1 = QtWidgets.QPushButton('Eve Dön', self)
        btn1.move(30, 50)
        btn1.resize(100, 50)
        btn1.clicked.connect(lambda: self.sendCommand(1))

        # Görüntü Al Butonu
        btn2 = QtWidgets.QPushButton('Görüntü Al', self)
        btn2.move(150, 50)
        btn2.resize(100, 50)
        btn2.clicked.connect(lambda: self.sendCommand(2))

        # Hızlan Butonu
        btn3 = QtWidgets.QPushButton('Hızlan', self)
        btn3.move(270, 50)
        btn3.resize(100, 50)
        btn3.clicked.connect(lambda: self.sendCommand(3))

        # Eve Dön butonu fonksiyonu
        def eve_don(self):
            send_command(0x01)  # 0x01 değeri "Eve Dön" butonunun numarasıdır.

        # Görüntü Al butonu fonksiyonu
        def goruntu_al(self):
            send_command(0x02)  # 0x02 değeri "Görüntü Al" butonunun numarasıdır.

        # Hızlan butonu fonksiyonu
        def hizlan(self):
            send_command(0x03)  # 0x03 değeri "Hızlan" butonunun numarasıdır.

        self.setGeometry(30, 20, 400, 150)
        self.setWindowTitle('Araç Kontrol Arayüzü')

class PlotWidget(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        self.init_ui(title)

    def init_ui(self, title):
        # Create plot widget
        self.plot_widget = pg.PlotWidget(title=title)

        # Create plot curve
        self.plot_curve = self.plot_widget.plot()

        # Create layout
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(self.plot_widget)
        # Set layout
        self.setLayout(vbox)

    def plot(self, value):
        # Get current data from plot curve
        x_data, y_data = self.plot_curve.getData()

        # Add new data to plot curve
        x_data = pg.np.append(x_data, x_data[-1] + 1)
        y_data = pg.np.append(y_data, value)
        self.plot_curve.setData(x=x_data, y=y_data)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window2 = SecondWindow()
    serial_port = SerialPort()
    window.show()
    window2.show()
    serial_port.show()
    sys.exit(app.exec())

