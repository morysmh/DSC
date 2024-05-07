import serial
import crc
import time
COM = 'COM11'
baud = 115200
firstrun = 0
ser = []
Ecom = dict(
    PC_SETTOGO = 1,
    PC_SetSpeed = 2,
    PC_MoveMotor = 3,
    PC_MoveAll = 4,
    PC_SetDefaulSpeed = 5,
    PC_DisableMotor = 6,
    PC_EnableMotor = 7,
    PC_GoHome = 8,
    PC_PersiceHome = 9,
    PC_RecoverMotor = 10)
def openSerialPort(comnumber = COM):
    global ser
    ser = serial.Serial(port=comnumber,baudrate=baud)
    firstmotorRun()

def firstmotorRun():
    global firstrun
    if firstrun == 1:
        return
    print("First Time Runned")
    #disableALL()
    rs485_send([(10)])
    firstrun = 1

def nmPefFullStep(mot,nanoMeter):
    pass


def rs485_send(data):
    global ser
    if(not ser):
        return
    ser.write(bytes(data))
    print(data)
    time.sleep(.1)

def sendNewCommand(motor,reg,val):
    valin = int(val)
    b = [(motor & 0xFF),(reg & 0xFF),((valin>>24) & 0xFF),((valin>>16) & 0xFF),((valin>>8) & 0xFF),(valin & 0xFF)]
    rs485_send(b)

def moveall():
    sendNewCommand(motor=1,reg=Ecom["PC_MoveAll"],val = 1)

def recoverMotor(mot):
    sendNewCommand(motor=mot,reg=Ecom["PC_RecoverMotor"],val = 1)

def startMoving(mot):
    sendNewCommand(motor=mot,reg=Ecom["PC_MoveMotor"],val = 1)

def moveMotor(mot,nanoMeter):
    sendNewCommand(motor=mot,reg=Ecom["PC_SETTOGO"],val = nanoMeter/10)

def speedMotor(mot,speed):
    #speed = 5000 - speed
    if speed < 10:
        speed = 10
    sendNewCommand(motor=mot,reg=Ecom["PC_SetSpeed"],val = speed)

def homeMotor(mot):
    sendNewCommand(motor=mot,reg=Ecom["PC_GoHome"],val = 1)
    
def EnableMotor(mot):
    sendNewCommand(motor=mot,reg=Ecom["PC_EnableMotor"],val = 1)

def disableMotor(mot):
    sendNewCommand(motor=mot,reg=Ecom["PC_DisableMotor"],val = 1)

def disableALL():
    for i in range(0,7):
        mot = i
        disableMotor(mot)

def stopCommunication():
    global ser
    if(not ser):
        print("No Port has Oppend")
        return
    ser.close()
    print("Communication Has Stoped")

def stpeedmot(mot, speed):
    speedMotor(mot,speed)

def readcomm():
    global ser
    if(not ser):
        return ""
    out = ""
    if(ser.readable()):
        out = ser.read_all().decode()
    #print(out)
    return out
