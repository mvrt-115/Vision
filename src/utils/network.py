import threading
from networktables import NetworkTables
import logging

# Setup

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)

    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='10.1.15.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

# Processing code

print("Connected!")

sd = NetworkTables.getTable("SmartDashboard")

# fe = fudicial estimated
def updatePose(x, y, theta):
    sd.putNumber("fe x", x)
    sd.putNumber("fe y", y)
    sd.putNumber("fe theta", theta)

def enableLogging(flag):
    logLevel = logging.CRITICAL

    if flag:
        logLevel = logging.DEBUG

    logging.basicConfig(level=logLevel)