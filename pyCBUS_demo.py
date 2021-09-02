# pyCBUS_demo.py
# A small demonstration program to show how to use pyCBUS module

import pyCBUS
import time
import threading

CBUSMessage=0
StopThread=False

# Dedicated thread function to process incoming CBUS messages
def CBUS_receive_func ():
    global StopThread
    while True:
        CBUSMessage = pyCBUS.getNextCBUSMessage()
        # pyCBUS.dumpCBUSMessage (CBUSMessage)   # Uncomment this line to show incoming CBUS message
        time.sleep (0.001)    # Needed if we don't want Python to take 100% of the CPU...
        if StopThread:
            break             # Exit the thread function and stop thread
           
def listCommands ():
    print ('0 : Exit program')
    print ('1 : Set track power on')
    print ('2 : Set track power off')
    print ('3 : Emergency stop')
    print ('4 : Test some accessory events')

# Small demo function to show how to send accessory events
def testAccessoryEvents ():
    # Event with no data bytes
    pyCBUS.accessoryEventLong (node_number=1234, event_number=1, isON=False, data=None)
    # Event with one data byte
    testData1 = [200]
    pyCBUS.accessoryEventLong (node_number=1234, event_number=1, isON=True, data=testData1)
    # Event with two data bytes
    testData2 = [100, 200]
    pyCBUS.accessoryEventLong (node_number=1234, event_number=1, isON=False, data=testData2)
    # Event with three data bytes
    testData3 = [10, 20, 30]
    pyCBUS.accessoryEventLong (node_number=1234, event_number=1, isON=True, data=testData3)

# Configure CANPiCAP and activate pyCBUS
pyCBUS.setCBUS_ID(126)
pyCBUS.activateSocketCAN()
pyCBUS.setup()
pyCBUS.setGreenLED (state=True)
pyCBUS.setYellowLED (state=False)
pyCBUS.setRedLED (state=False)

# Create CBUS reception thread
CBUSThread = threading.Thread (target=CBUS_receive_func, args=())
CBUSThread.start()

# Command loop
Command = -1
while Command != 0:
    listCommands()
    Command = int (input ('Enter command '))
    
    if Command == 1:
        pyCBUS.setGreenLED (state=True)
        pyCBUS.setTrackPower (power_on=True)
    elif Command == 2:
        pyCBUS.setGreenLED (state=False)
        pyCBUS.setTrackPower (power_on=False)
    elif Command == 3:
        pyCBUS.emergencyStop()
    elif Command == 4:
        testAccessoryEvents()

# Program termination
StopThread = True
CBUSThread.join()    # This will kill the thread (can be omitted, but better practice)