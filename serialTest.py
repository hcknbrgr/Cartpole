import serial
import time
import matplotlib.pyplot as plt

ser = serial.Serial('COM7', 9800, timeout=.5)
time.sleep(2)

data = []
for i in range(50):
    line = ser.readline()
    if line:
        string = line.decode()
        num = int(string)
        print(num)
        data.append(num)

ser.close()

plt.plot(data)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.title('Time vs Distance')
plt.show()