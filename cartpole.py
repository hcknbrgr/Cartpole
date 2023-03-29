import serial
import time
import struct  # for sending data as bytes
import matplotlib.pyplot as plt

arduino = serial.Serial('COM7', 9800, write_timeout=0.5, timeout=.5)
time.sleep(2)

data = []
for i in range(50):
    line = arduino.readline()
    if line:
        string = line.decode()
        num = int(string)
        print(num)
        if num > 12:
            arduino.write(struct.pack('>B', 1))
        else:
            arduino.write(struct.pack('>B', 0))
        data.append(num)

arduino.close()

plt.plot(data)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.title('Time vs Distance')
plt.show()