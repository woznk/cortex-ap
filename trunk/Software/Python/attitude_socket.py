from visual import *
from struct import *
import socket
import string
import math
import numpy

UDP_IP="127.0.0.1"
UDP_PORT=5005

sock = socket.socket( socket.AF_INET,     # Internet
		      socket.SOCK_DGRAM ) # UDP
sock.bind( (UDP_IP,UDP_PORT) )

scene.range=(2,2,1)
scene.forward = (1.0,0.0,0.3)
scene.up=(0,0,-1)
scene.lights = []
lamp=distant_light(direction=(0.5, 0.5, 1), color=color.gray(0.7)) 
lamp=distant_light(direction=(-0.5, -0.5, -1), color=color.gray(1))

arrow(color=color.green,axis=(2,0,0), shaftwidth=0.02, fixedwidth=1)
arrow(color=color.yellow,axis=(0,2,0), shaftwidth=0.02 , fixedwidth=1)
arrow(color=color.blue,axis=(0,0,1), shaftwidth=0.02, fixedwidth=1)

wing = box(size=(.5,.05,3), color=color.yellow)
fuse = box(size=(2,.2,.2), color=color.green)
#fuse = arrow(color=color.green)

mylabel = label(pos=(-.5,0,1),height=10,box=0)
hexadecimal = "0123456789ABCDEF"

rm = numpy.zeros(shape = (3,3))

while True:
    data, addr = sock.recvfrom( 37 )    # buffer size is 37 bytes

    for y in range (3):
        for x in range (3):
            k = (x * 4) + (y * 12)
            rm[y][x] = unpack_from('<f', data, k)[0]
    axis = (rm[0][0], rm[1][0], rm[2][0])
    up   = (rm[0][2], rm[1][2], rm[2][2])
                    
    wing.axis=axis
    wing.up=up
    wing.size=(.5,.05,3)
    fuse.axis=axis
    fuse.up=up
    fuse.size=(2,.2,.2)
    
    message =  str(rm[0][0])+ " " + str(rm[0][1])+ " " + str(rm[0][2])+ "\n"
    message += str(rm[1][0])+ " " + str(rm[1][1])+ " " + str(rm[1][2])+ "\n"
    message += str(rm[2][0])+ " " + str(rm[2][1])+ " " + str(rm[2][2])+ "\n"
    mylabel.text = message

sock.unbind( (UDP_IP,UDP_PORT) )

