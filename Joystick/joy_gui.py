import time
import serial
try:
    port = serial.Serial("/dev/ttyACM0", baudrate=115200, timeout=3.0)
    print(f"Opened serial: {port.name}")

except Exception as e:
    print(f"error open serial port: {str(e)}")

string = "hi"
while True:

    # port.write(string.encode())
    # rcv = port.read(100)
    data = port.readline().decode('utf-8').strip()
    print(data)
    # time.sleep(0.1)