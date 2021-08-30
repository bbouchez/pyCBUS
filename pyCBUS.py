# pyCBUS.py
# MERG CBUS Python module for CANPiCAP (Kit86) interface
#
# Copyright 2021 Benoit BOUCHEZ (M8718) 
# Creative Commons Attribution-NonCommercial-ShareaLIKE 4.0 International License
#  License summary:
#    You are free to:
#      Share, copy and redistribute the material in any medium or format
#      Adapt, remix, transform, and build upon the material
#    The licensor cannot revoke these freedoms as long as you follow the license terms.
#    Attribution : You must give appropriate credit, provide a link to the license,
#                   and indicate if changes were made. You may do so in any reasonable manner,
#                   but not in any way that suggests the licensor endorses you or your use.
#    NonCommercial : You may not use the material for commercial purposes. **(see note below)
#    ShareAlike : If you remix, transform, or build upon the material, you must distribute
#                  your contributions under the same license as the original.
#    No additional restrictions : You may not apply legal terms or technological measures that
#                                  legally restrict others from doing anything the license permits.
#   ** For commercial use, please contact the original copyright holder(s) to agree licensing terms
#
#    This software is distributed in the hope that it will be useful, but WITHOUT ANY
#    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE

import can
import os
import RPi.GPIO as GPIO

# CBUS opcodes
OPC_ACK = 0x00     #  General affirmative acknowledge
OPC_NAK = 0x01     #  General negative acknowledge
OPC_HLT = 0x02     #  CAN bus not available / busy
OPC_BON = 0x03     #  CAN bus available
OPC_TOF = 0x04     #  DCC track is off
OPC_TON = 0x05     #  DCC track is on
OPC_ESTOP = 0x06   #  Emergency stop all
OPC_ARST = 0x07    #  System reset
OPC_RTOF = 0x08    #  Request track off
OPC_RTON = 0x09    #  Request track on
OPC_RESTP = 0x0A   #  Request emergency stop all
OPC_RSTAT = 0x0C   #  Query status of command station
OPC_QNN = 0x0D     #  Query node status
OPC_RQNP = 0x10    #  Request node parameters
OPC_RQMN = 0x11    #  Request module name
OPC_KLOC = 0x21    #  Release engine
OPC_QLOC = 0x22    #  Query engine
OPC_DKEEP = 0x23   #  Session keepalive from CAB
OPC_DBG1 = 0x30    #  Debug. For development only
OPC_EXTC = 0x3F    #  Extended OPC with no added bytes
OPC_RLOC = 0x40    #  Request engine session
OPC_QCON = 0x41    #  Query consist
OPC_SNN = 0x42     #  Set node number
OPC_ALOC = 0x43    #  Allocate loc to assignment or activity
OPC_STMOD = 0x44   #  Set cab session mode
OPC_PCON = 0x45    #  Set loco into consist
OPC_KCON = 0x46    #  Remove loco from consist
OPC_DSPD = 0x47    #  Set engine speed / direction
OPC_DFLG = 0x48    #  Set engine session flags
OPC_DFNON = 0x49   #  Set engine function ON
OPC_DFNOF = 0x4A   #  Set engine function OFF
OPC_SSTAT = 0x4C   #  Service mode status
OPC_RQNN = 0x50    #  Request node number
OPC_NNREL = 0x51   #  Node number release
OPC_NNACK = 0x52   #  Node number acknowledge
OPC_NNLRN = 0x53   #  Set node into learn mode
OPC_NNULN = 0x54   #  Release node from learn mode
OPC_NNCLR = 0x55   #  Clear all events from a node
OPC_NNEVN = 0x56   #  Read number of events available
OPC_NERD = 0x57    #  Read back all events in a node
OPC_RQEVN = 0x58   #  Read number of stored events in a node
OPC_WRACK = 0x59   #  Write acknowledge
OPC_RQDAT = 0x5A   #  Request node data event
OPC_RQDDS = 0x5B   #  Request device data (short)
OPC_BOOTM = 0x5C   #  Put node into bootloader mode
OPC_ENUM = 0x5D    #  Force self enumeration of CAN_ID
OPC_EXTC1 = 0x5F   #  Extended OPC with one added byte
OPC_DFUN = 0x60    #  Set engine functions (DCC format)
OPC_GLOC = 0x61    #  Get engine session
OPC_ERR = 0x63     #  Command station error report
OPC_CMDERR = 0x6F  #  Error message during configuration
OPC_EVNLF = 0x70   #  Event space left
OPC_NVRD = 0x71    #  Request read of node variable
OPC_NENRD = 0x72   #  Request read of events by index
OPC_RQNPN = 0x73   #  Request read of node parameter by index
OPC_NUMEV = 0x74   #  Number of events stored in node
OPC_CANID = 0x75   #  For a specific CAN_ID
OPC_EXTC2 = 0x7F   #  Extended OPC with two added bytes
OPC_RDCC3 = 0x80   #  Request 3 byt DCC packet
OPC_WCVO = 0x82    #  Write CV in OPS mode (byte)
OPC_WCVB = 0x83    #  Write CV in OPS mode (bit)
OPC_QCVS = 0x84    #  Request read CV (service mode)
OPC_PCVS = 0x85    #  Report CV (service mode)
OPC_ACON = 0x90    #  Accessory ON event (long)
OPC_ACOF = 0x91    #  Accessory OFF event (long)
OPC_AREQ = 0x92    #  Accessory status request (long)
OPC_ARON = 0x93    #  Accessory response ON (long)
OPC_AROF = 0x94    #  Accessory response OFF (long)
OPC_EVULN = 0x95   #  Unlearn an event in learn mode
OPC_NVSET = 0x96   #  Set a node variable
OPC_NVANS = 0x97   #  Node variable value response
OPC_ASON = 0x98    #  Accessory ON event (short)
OPC_ASOF = 0x99    #  Accessory OFF event (short)
OPC_ASRQ = 0x9A    #  Accessory status request (short)
OPC_PARAN = 0x9B   #  Parameter readback by index
OPC_REVAL = 0x9C   #  Request read of event variable
OPC_ARSON = 0x9D   #  Accessory response ON (short)
OPC_ARSOF = 0x9E   #  Accessory response OFF (short)
OPC_EXTC3 = 0x9F   #  Extended OPC with three added bytes
OPC_RDCC1 = 0xA0   #  Request 4 byte DCC packet
OPC_WCVS = 0xA2    #  Write CV in service mode
OPC_ACON1 = 0xB0   #  Accessory ON event with one added byte (long)
OPC_ACOF1 = 0xB1   #  Accessory OFF event with one added byte (long)
OPC_REQEV = 0xB2   #  Read event variable in learn mode
OPC_ARON1 = 0xB3   #  Accessory response event ON with one added byte (long)
OPC_AROF1 = 0xB4   #  Accessory response event OFF with one added byte (long)
OPC_NEVAL = 0xB5   #  Read of EV value response
OPC_PNN = 0xB6     #  Response to query node
OPC_ASON1 = 0xB8   #  Accessory ON event with one added byte (short)
OPC_ASOF1 = 0xB9   #  Accessory OFF event with one added byte (short)
OPC_ARSON1 = 0xBD  #  Accessory response ON with one added byte (short)
OPC_ARSOF1 = 0xBE  #  Accessory response OFF with one added byte (short)
OPC_EXTC4 = 0xBF   #  Extended OPC with four added bytes
OPC_RDCC5 = 0xC0   #  Request 5 byte DCC packet
OPC_WCVOA = 0xC1   #  Write CV in OPS mode by address
OPC_FCLK = 0xCF    #  Fast clock
OPC_ACON2 = 0xD0   #  Accessory ON event with two added byte (long)
OPC_ACOF2 = 0xD1   #  Accessory OFF event with two added byte (long)
OPC_EVLRN = 0xD2   #  Teach event in learn mode
OPC_EVANS = 0xD3   #  Response to request for EV value in learn mode
OPC_ARON2 = 0xD4   #  Accessory response event ON with two added byte (long)
OPC_AROF2 = 0xD5   #  Accessory response event OFF with two added byte (long)
OPC_ASON2 = 0xD8   #  Accessory ON event with two added byte (short)
OPC_ASOF2 = 0xD9   #  Accessory OFF event with two added byte (short)
OPC_ARSON2 = 0xDD  #  Accessory response ON with two added byte (short)
OPC_ARSOF2 = 0xDE  #  Accessory response OFF with two added byte (short)
OPC_EXTC5 = 0xDF   #  Extended OPC with five added bytes
OPC_RDCC6 = 0xE0   #  Request 6 byte DCC packet
OPC_PLOC = 0xE1    #  Engine report from command station
OPC_NAME = 0xE2    #  Response to request for node name
OPC_STAT = 0xE3    #  Command station status report
OPC_PARAMS = 0xEF  #  Response to request for node parameters (in setup)
OPC_ACON3 = 0xF0   #  Accessory ON event with three added byte (long)
OPC_ACOF3 = 0xF1   #  Accessory OFF event with three added byte (long)
OPC_ENRSP = 0xF2   #  Response to request to read node events
OPC_ARON3 = 0xF3   #  Accessory response event ON with three added byte (long)
OPC_AROF3 = 0xF4   #  Accessory response event OFF with three added byte (long)
OPC_EVLRNI = 0xF5  #  Teach event in learn mode using event indexing
OPC_ACDAT = 0xF6   #  Accessory node data event 5 data bytes (long)
OPC_ARDAT = 0xF7   #  Accessory node data response 5 data bytes (long)
OPC_ASON3 = 0xF8   #  Accessory ON event with three added byte (short)
OPC_ASOF3 = 0xF9   #  Accessory OFF event with three added byte (short)
OPC_DDES = 0xFA    #  Accessory node data event 5 data bytes (short)
OPC_DDRS = 0xFB    #  Accessory node data response 5 data bytes (short)
OPC_ARSON3 = 0xFD  #  Accessory response ON with three added byte (short)
OPC_ARSOF3 = 0xFE  #  Accessory response OFF with three added byte (short)
OPC_EXTC6 = 0xBF   #  Extended OPC with six added bytes

# CANPiCAP GPIO usage
PUSH_BUTTON = 17
RED_LED = 22
YELLOW_LED = 23
GREEN_LED = 24

# *** Global variables ***
# CBUS CAN identifier configuration
CBUS_ID = 0x2FF           # value is 0 to 2047
CBUS_MAJOR_PRIORITY = 0   # value is 0 (max) to 2 (3 is not allowed)
CBUS_MINOR_PRIORITY = 0   # value is 0 (max) to 3
bus = 0

# *** MODULE CONFIGURATION ***
# Configure the RPi GPIO and CAN adapter
def setup ():
    global bus
    bus = can.interface.Bus (channel = 'can0', bustype = 'socketcan')
    GPIO.setmode (GPIO.BCM)   #  Use BCM processor pin numbering (GPIOxx)
    GPIO.setwarnings(False)
    GPIO.setup (PUSH_BUTTON, GPIO.IN)
    GPIO.setup (RED_LED, GPIO.OUT)
    GPIO.setup (YELLOW_LED, GPIO.OUT)
    GPIO.setup (GREEN_LED, GPIO.OUT)
    
# Activate can0 socket (sudo is allowed like this on RPi)
# This function should return 0 when can0 adapter is created successfully
# can0 socket an also be activated automatically when RPi is starting :
# Add the following lines in /etc/network/interfaces
# auto can0
# iface can0 inet manual
# pre-up /sbin/ip link set can0 type can bitrate 125000 triple-sampling on restart-ms 100
# up /sbin/ifconfig can0 up
# down /sbin/ifconfig can0 down
def activateSocketCAN ():
    # Deactivate can0 interface before activating it
    # if can0 is already active, a warning message is displayed if
    # we try to activate it again
    os.system ('sudo ifconfig can0 down')
    # Activate can0 with CBUS baudrate
    return os.system ('sudo ip link set can0 up type can bitrate 125000')
    
# *** CANPICAP LED AND BUTTON CONTROL ***
# Control of CANPiCAP red LED D4
def setRedLED (state):
    GPIO.output (RED_LED, state)
    
# Control of CANPiCAP yellow LED D3
def setYellowLED (state):
    GPIO.output (YELLOW_LED, state)
    
# Control of CANPiCAP green LED D5
def setGreenLED (state):
    GPIO.output (GREEN_LED, state)

# Return state of S1 pushbutton
def isS1Depressed ():
    if GPIO.input (PUSH_BUTTON) == False:
        return True;
    else:
        return False;

# Set CAN identifier used in CBUS messages
def setCBUS_ID (can_id):
    global CBUS_ID
    CBUS_ID=can_id+(CBUS_MAJOR_PRIORITY<<9)+(CBUS_MINOR_PRIORITY<<7)

# Get next CBUS message in reception queue
# Function is non blocking and returns None if no CAN message has been received
def getNextCBUSMessage ():
    msg = bus.recv (0)    # 0 : Make bus.recv non blocking
    return msg

# Utility function to display CBUS message dump from a Python-CAN Msg
def dumpCBUSMessage (msg):
    if msg == None:
        return
    print ('CBUS ID:', msg.arbitration_id, 'DLC:', msg.dlc, 'Data:', ' '.join(format(x, '02x') for x in msg.data))

# *** CBUS control messages ***
def setTrackPower (power_on):
    if power_on == True:
        msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_RTON], is_extended_id=False)
    else:
        msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_RTOF], is_extended_id=False)        
    bus.send(msg)    

def requestSession (dcc_loc_number):
    if dcc_loc_number > 10239:    # 0x27FF is the highest address allowed by NMRA
        return
    if dcc_loc_number <= 127:
        msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_RLOC, 0, dcc_loc_number], is_extended_id=False)        
    # TODO : add support for long address
    bus.send(msg)
    
def releaseSession (session):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_KLOC, session], is_extended_id=False)
    bus.send(msg)
    
# speed_mode : 0 to 3
# service_mode : False / True
# sound_control_mode : False / True
def setCABSessionMode (session, speed_mode, service_mode = False, sound_control_mode = False):
    if speed_mode > 3:  # check illegal value for speed_mode
        return
    Dat2 = speed_mode   # set bits 0 / 1
    if service_mode == True:
        Dat2 = Dat2 + 4   # set bit 2
    if sound_control_mode == True:
        Dat2 = Dat2 + 8   # set bit 3
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_STMOD, session, Dat2], is_extended_id=False)
    bus.send(msg)
    
def keepAliveSession (session):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_DKEEP, session], is_extended_id=False)
    bus.send(msg)
    
def setSpeedAndDirection (session, speed, forward):
    Dat2 = speed
    if forward == True:
        Dat2 = Dat2 + 0x80   # set forward direction flag
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_DSPD, session, Dat2], is_extended_id=False)
    bus.send(msg)
    
def emergencyStop ():
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_RESTP], is_extended_id=False)
    bus.send(msg)
    
# *** LOCO DECODER FUNCTIONS ***
def setEngineFunction (session, function, activate):
    if activate == True:
        msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_DFNON, session, function], is_extended_id=False)
    else:
        msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_DFNOF, session, function], is_extended_id=False)
    bus.send(msg)    
    
# *** NODE VARIABLES READ/WRITE ***
def setNodeVariable (node_number, variable_number, variable_value):
    if variable_number < 1 or variable_number > 255:
        return
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NVSET, node_number>>8, node_number&0xFF, \
                                                     variable_number, variable_value], is_extended_id=False)
    bus.send(msg)
    
def readNodeVariable (node_number, variable_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NVRD, node_number>>8, node_number&0xFF, \
                                                     variable_number], is_extended_id=False)
    bus.send(msg)

# *** TEACHING EVENTS AND EVENT VARIABLES ***
def activateLearnMode (node_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NNLRN, node_number>>8, node_number&0xFF], is_extended_id=False)
    bus.send(msg)
    
# For teaching device numbers using device addressing, node_number must be 0. event_number shall contain then the device address
def sendEventToLearn (node_number, event_number, event_variable, event_value):
    if event_variable < 1  or event_variable > 255 :
        return
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_EVLRN, node_number>>8, node_number&0xFF, \
                                                     event_number>>8, event_number&0xFF, \
                                                     event_variable, event_value], is_extended_id=False)
    bus.send(msg)
    
def exitLearnMode (node_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NNULN, node_number>>8, node_number&0xFF], is_extended_id=False)
    bus.send(msg)
    
def removeEvent (node_number, event_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_EVULN, node_number>>8, node_number&0xFF, \
                                                     event_number>>8, event_number&0xFF], is_extended_id=False)
    bus.send(msg)

# *** ACCESSORY EVENT REQUEST ***
def accessoryRequestEventLong (node_number, event_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_AREQ, node_number>>8, node_number&0xFF, event_number>>8, \
                                                     event_number&0xFF], is_extended_id=False)
    bus.send(msg)
    
def accessoryRequestEventShort (node_number, device_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASRQ, node_number>>8, node_number&0xFF, device_number>>8, \
                                                     device_number&0xFF], is_extended_id=False)
    bus.send(msg)
    
# data can be None or array of 1, 2 or 3 data bytes
def accessoryEventLong (node_number, event_number, isON, data):
    if data == None:
        if isON == False:
            msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACOF, node_number>>8, node_number&0xFF, \
                                                             event_number>>8, event_number&0xFF], \
                               is_extended_id=False)
        else:
            msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACON, node_number>>8, node_number&0xFF, \
                                                             event_number>>8, event_number&0xFF], \
                               is_extended_id=False)           
    else:
        datalen = len(data)
        if datalen == 1:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACOF1, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, data[0]], \
                                   is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACON1, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, data[0]], \
                                   is_extended_id=False)           
            
        elif datalen == 2:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACOF2, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, \
                                                                 data[0], data[1]], is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACON2, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, \
                                                                 data[0], data[1]], is_extended_id=False)           

        elif datalen == 3:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACOF3, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, \
                                                                 data[0], data[1], data[2]], \
                                   is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ACON3, node_number>>8, node_number&0xFF, \
                                                                 event_number>>8, event_number&0xFF, data[0], \
                                                                 data[1], data[2]], \
                                   is_extended_id=False)           
        
        else:
            return    # Invalid number of bytes
    bus.send (msg)

def accessoryEventShort (node_number, device_number, isON, data):
    if data == None:
        if isON == False:
            msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASOF, node_number>>8, node_number&0xFF, \
                                                             device_number>>8, device_number&0xFF], \
                               is_extended_id=False)
        else:
            msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASON, node_number>>8, node_number&0xFF, \
                                                             device_number>>8, device_number&0xFF], \
                               is_extended_id=False)           
    else:
        datalen = len(data)
        if datalen == 1:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASOF1, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, data[0]], \
                                   is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASON1, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, data[0]], \
                                   is_extended_id=False)           
            
        elif datalen == 2:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASOF2, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, \
                                                                 data[0], data[1]], is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASON2, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, \
                                                                 data[0], data[1]], is_extended_id=False)           

        elif datalen == 3:
            if isON == False:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASOF3, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, \
                                                                 data[0], data[1], data[2]], \
                                   is_extended_id=False)
            else:
                msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_ASON3, node_number>>8, node_number&0xFF, \
                                                                 device_number>>8, device_number&0xFF, \
                                                                 data[0], data[1], data[2]], \
                                   is_extended_id=False)           
        
        else:
            return    # Invalid number of bytes
    bus.send (msg)
    
# *** MISCELLANEOUS ***
def queryAllNodes ():
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_QNN], is_extended_id=False)
    bus.send(msg)
    
def readAllEvents (node_number):
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NERD, node_number>>8, node_number&0xFF], is_extended_id=False)
    bus.send(msg)
                       
def readEventFromNumber (node_number, event_number):
    if event_number < 0 or event_number > 255:
        return
    msg = can.Message (arbitration_id=CBUS_ID, data=[OPC_NENRD, node_number>>8, node_number&0xFF, \
                                                     event_number], is_extended_id=False)
    bus.send(msg)