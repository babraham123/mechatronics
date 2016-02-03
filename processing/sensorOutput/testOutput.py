#!/usr/bin/python
import serial, time

# /dev/tty./dev/cu.usbmodem1421
s = serial.Serial(port='/dev/tty.usbmodem1421', baudrate=9600)
count = 0
while True:
    time.sleep(0.1)
    count = (count + 1) % 255
    s.write(str(count))


print 'Done!'
# s.read()
# s.readline()