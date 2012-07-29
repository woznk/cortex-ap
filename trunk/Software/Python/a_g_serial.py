from visual import *
import serial
import string
import math
import numpy

ser = serial.Serial('COM6', 115200, timeout=1)

scene.range=(2,2,1)
scene.forward = (1.0,0.0,0.3)
scene.up=(0,0,-1)
scene.lights = []
lamp=distant_light(direction=(0.5, 0.5, 1), color=color.gray(0.7)) 
lamp=distant_light(direction=(-0.5, -0.5, -1), color=color.gray(1))

acc = arrow(color=color.yellow, axis = (0,0,0), shaftwidth=0.02, fixedwidth=1)
rate = arrow(color=color.red, axis = (0,0,0), shaftwidth=0.02, fixedwidth=1)
label = label(pos=(0,0,-1),height=10,box=0)
hexadecimal = "0123456789ABCDEF"
rm = numpy.zeros(shape = (6))

while 1:
    line = ser.readline()
    Words = string.split(line)

    if len(Words) == 6:
        error = 0
        for k in range (6):
            if len(Words[k]) == 4:
                for j in range (4):
                    error += (Words[k][j] not in hexadecimal)
                if error == 0:
                    temp = int(Words[k],16)
                    if temp < 32768:
                        temp = temp / 32768.
                    else:
                        temp = (temp - 65536) / 32768.
                    rm[k] = temp
            else:
                error = 1
        if error == 0:
            acc.axis = (rm[0] * 100, rm[1] * 100, rm[2] * 100)
            rate.axis = (rm[3], rm[4], rm[5])
            label.text = "A " + str(rm[0]) + "\n  " + str(rm[1]) + "\n  " + str(rm[2]) + "\nG " + str(rm[3]) + "\n  " + str(rm[4]) + "\n  " + str(rm[5])

ser.close

