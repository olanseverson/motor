# -*- coding: utf-8 -*-
"""
Created on Wed Sep 18 22:50:04 2019

@author: olanseverson
"""

# Uncomment the next two lines if you want to save the animation
#import matplotlib
#matplotlib.use("Agg")

from threading import Thread
import time
import serial
import numpy
from matplotlib.pylab import *
from mpl_toolkits.axes_grid1 import host_subplot
import matplotlib.animation as animation

class serialPlot:
    def __init__(self, serialPort='/dev/ttyACM0', serialBaud=115200):
        self.port = serialPort
        self.baud = serialBaud
        self.rawData = bytearray(16)
        self.isReceiving = False
        self.thread = None
        self.isRun = True
 
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=.1)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
 
    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
    
    def splitData(self):
        return str(self.parsedData).split(',')
    
    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.rawData = self.serialConnection.readline()[:-2] #the last bit gets rid of the new-line chars
            if self.rawData:
#                print(self.rawData)
                self.isReceiving = True
                self.parsedData = self.rawData.decode()[:-1]
#                bf_parsed = self.rawData.decode()
#                after_parsed = bf_parsed.split('/')
#                for i in after_parsed:
#                    print (i)
 
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')

# Sent for figure
font = {'size'   : 9}
matplotlib.rc('font', **font)

# Setup figure and subplots
f0 = figure(num = 0, figsize = (12, 8))#, dpi = 100)
f0.suptitle("Gait Analysis", fontsize=12)
ax01 = subplot2grid((2, 2), (0, 0))
ax02 = subplot2grid((2, 2), (0, 1))
ax03 = subplot2grid((2, 2), (1, 0))
ax04 = subplot2grid((2, 2), (1, 1))
#ax04 = ax03.twinx()
#tight_layout()

# Set titles of subplots
ax01.set_title('Right Hip')
ax02.set_title('Left Hip')
ax03.set_title('Right Knee')
ax04.set_title('Left Knee')

# set y-limits
ax01.set_ylim(-15,45)
ax02.set_ylim(-15,45)
ax03.set_ylim(-60,10)
ax04.set_ylim(-60,10)

# sex x-limits
ax01.set_xlim(0,15.0)
ax02.set_xlim(0,15.0)
ax03.set_xlim(0,15.0)
ax04.set_xlim(0,15.0)

# Turn on grids
ax01.grid(True)
ax02.grid(True)
ax03.grid(True)
ax04.grid(True)

# set label names
ax01.set_xlabel("time (s)")
ax01.set_ylabel("angle (deg)")
ax02.set_xlabel("time (s)")
ax02.set_ylabel("angle (deg)")
ax03.set_xlabel("time (s)")
ax03.set_ylabel("angle (deg)")
ax04.set_xlabel("time (s)")
ax04.set_ylabel("angle (deg)")

# Data Placeholders
arh=zeros(0)
trh=zeros(0)

alh=zeros(0)
tlh=zeros(0)

ark=zeros(0)
trk=zeros(0)

alk=zeros(0)
tlk=zeros(0)

t=zeros(0)

# set plots
p011, = ax01.plot(t,arh,'b-', label="Actual")
p012, = ax01.plot(t,trh,'g-', label="Target")

p021, = ax02.plot(t,alh,'b-', label="Actual")
p022, = ax02.plot(t,tlh,'g-', label="Target")

p031, = ax03.plot(t,ark,'b-', label="Actual")
p032, = ax03.plot(t,trk,'g-', label="Target")

p041, = ax04.plot(t,alk,'b-', label="Actual")
p042, = ax04.plot(t,tlk,'g-', label="Target")

# set lagends
ax01.legend([p011,p012], [p011.get_label(),p012.get_label()])
ax02.legend([p021,p022], [p021.get_label(),p022.get_label()])
ax03.legend([p031,p032], [p031.get_label(),p032.get_label()])
ax04.legend([p041,p042], [p041.get_label(),p042.get_label()])

# Data Update
xmin = 0.0
xmax = 15.0
x = 0.0

portName = '/dev/ttyACM0'
baudRate = 115200
s = serialPlot(portName, baudRate)   # initializes all required variables
s.readSerialStart() 

def updateData(self):
    global x
    global arh
    global trh
    global alh
    global tlh
    global ark
    global trk
    global alk
    global tlk
    global t

    tmpp1 = 1 + exp(-x) *sin(2 * pi * x)
    tmpv1 = - exp(-x) * sin(2 * pi * x) + exp(-x) * cos(2 * pi * x) * 2 * pi
    
#     Ubah append dengan nilai hasil bacaan serial untuk masing-masing gait
#    arh=append(arh,tmpp1)
#    trh=append(trh,tmpv1)
#    alh=append(alh,5*tmpp1)
#    tlh=append(tlh,5*tmpv1)
#    ark=append(ark,2.5*tmpp1)
#    trk=append(trk,2.5*tmpv1)
#    alk=append(alk,7.5*tmpp1)
#    tlk=append(tlk,7.5*tmpv1)
    
    data = s.splitData()
#    data = [1,2,3,4,5,6,7,8]
    arh=append(arh,data[0])
    trh=append(trh,data[1])
    alh=append(alh,data[2])
    tlh=append(tlh,data[3])
    ark=append(ark,data[4])
    trk=append(trk,data[5])
    alk=append(alk,data[6])
    tlk=append(tlk,data[7])
    t=append(t,x)

    x += 0.15

    p011.set_data(t,arh)
    p012.set_data(t,trh)

    p021.set_data(t,alh)
    p022.set_data(t,tlh)

    p031.set_data(t,ark)
    p032.set_data(t,trk)
    
    p041.set_data(t,alk)
    p042.set_data(t,tlk)

    if x >= xmax-3.00:
        p011.axes.set_xlim(x-xmax+3.0,x+3.0)
        p021.axes.set_xlim(x-xmax+3.0,x+3.0)
        p031.axes.set_xlim(x-xmax+3.0,x+3.0)
        p041.axes.set_xlim(x-xmax+3.0,x+3.0)

    return p011, p012, p021, p022, p031, p032, p041, p042

# interval: draw new frame every 'interval' ms
# frames: number of frames to draw
simulation = animation.FuncAnimation(f0, updateData, blit=False, interval=20, repeat=False)

# Uncomment the next line if you want to save the animation
#simulation.save(filename='sim.mp4',fps=30,dpi=300)

plt.show()

