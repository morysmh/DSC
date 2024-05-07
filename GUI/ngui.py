import tkinter as tk
from tkinter import *
import os
import sys
import glob
import serial
import Step
import re
import time
import guiclass
allmotor = []
portOpenCheck = 0
def serial_ports():
    """ Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

def convertPosLinear(pos):
    isneg = 0
    if(pos < 0):
        isneg = 1
        pos = pos * -1
    xx = "nm"
    if(pos > 1000):
        xx = "um"
        pos = pos / 1000
    if(pos > 1000):
        xx = "mm"
        pos = pos / 1000
    strval = str(pos) + xx
    if isneg:
        strval = '-' + str(pos) + xx
    return strval

def HomeMotor():
    mot2.set_Position_Absoulute(10*(10**6))
    mot3.HomeMotor(1)
    mot4.HomeMotor(1)

def zeroAllmotor():
    mot1.HomeMotor(1)
    mot2.HomeMotor(1)
    mot3.HomeMotor(1)
    mot4.HomeMotor(1)

def shutMotor():
    global allmotor
    for i in allmotor:
        i.shutmotor()


def openPort():
    global portOpenCheck 
    portOpenCheck = 1
    Step.openSerialPort(clicked.get())
    Step.firstmotorRun()
    global mot1,mot2,mot3,mot4
    mot3.maxspeed = 400
    mot1.maxspeed = 400
    mot2.maxspeed = 30
    mot4.maxspeed = 50
    #mot3.can_Go_Negetive()
    global allmotor
    for i in allmotor:
        i.enable_comnmunicatin()
        i.setspeed(i.maxspeed)


def startaction(event):
    global allmotor
    global portOpenCheck 
    for i in allmotor:
        i.send_Position_To_Step()
    if portOpenCheck == 0:
        return
    refresh_Txt_section()
    a = Step.readcomm()

def actionHome(event):
    HomeMotor()
auto_grinde = 0

def getPosition():
    global auto_grinde
    if(auto_grinde == 1):
        root.after(900,getPosition)              
        check_auto_grind_Command()

filecsv = open("mycsv.csv","r")

def check_auto_grind_Command():
    global auto_grinde
    global filecsv
    global allmotor
    if auto_grinde == 0:
        return 0
    if filecsv.closed:
        filecsv = open("mycsv.csv","r")
    isallzero = 0
    for i in allmotor:
        isallzero = isallzero + i.is_reachSetPos()
    if isallzero != 4:
        return 0
    b = filecsv.readline().split(',')
    if not b[0]:
        change_auto_stat()
        return 0
    tmpvar = [re.sub("[^0-9.-]","",i) + "u" for i in b]
    print(tmpvar)
    set_next_from_csv(tmpvar)
    return 1

def set_next_from_csv(indata):
    global allmotor
    global auto_grinde
    if auto_grinde == 0:
        return 0
    b = -1
    for i in allmotor:
        b = b + 1
        try:
            c = indata[b]
        except:
            c = 'u'
        if(c == 'u'):
            continue
        i.user_input_releative(c)
    for i in allmotor:
        i.send_Position_To_Step()
    return 1

def change_auto_stat():
    global auto_grinde
    global button_auto
    global filecsv
    if auto_grinde:
        auto_grinde = 0
        button_auto.config(bg="#263D42")
        if not filecsv.closed:
            filecsv.close()
    else:
        auto_grinde = 1
        button_auto.config(bg="#81e381")
        getPosition()



def printvarcurrposi(i_val):
    i_val = i_val.split(" ")
    motstat = 0
    motNum = 0
    valMot = 0
    try:
        motstat = int(re.sub("[^0-9]","",i_val[2]))
        motNum = int(re.sub("[^0-9]","",i_val[0]))
        valMot = int(re.sub("[^0-9-]","",i_val[1])) * 10
    except:
        return
    if(motNum == 1):
        mot1.set_motorStat(motstat)
        mot1.setmotorPos(convertPosLinear(valMot))
        mot1.set_read_position(valMot)
    if(motNum == 2):
        mot2.set_motorStat(motstat)
        mot2.setmotorPos(convertPosLinear(valMot))
        mot2.set_read_position(valMot)
    if(motNum == 3):
        mot3.set_motorStat(motstat)
        mot3.setmotorPos(convertPosLinear(valMot))
        mot3.set_read_position(valMot)
    if(motNum == 4):
        mot4.set_motorStat(motstat)
        mot4.setmotorPos(convertPosLinear(valMot))
        mot4.set_read_position(valMot)

def print_current_Pos(i_dat):
    if(i_dat == ""):
        return
    s_a = i_dat.split("\n")
    for i in s_a:
        printvarcurrposi(i)

def refresh_Txt_section():
    global root
    a = Step.readcomm()
    print_current_Pos(a)
    root.after(200,refresh_Txt_section)     

def closeport():
    Step.stopCommunication()

root = tk.Tk()

SerilList = serial_ports()
clicked = StringVar()
if(SerilList):
    clicked.set(SerilList[0])
else:
    clicked.set("Select Port")

basewidthGUI = 1400
canvas = tk.Canvas(root, height=800, width=basewidthGUI, bg="#253D42")
canvas.pack()
frame = tk.Frame(root, bg="white")
frame.place(width=basewidthGUI-basewidthGUI*.1, height=50,relx=0.05, rely=0.01)

drop = OptionMenu(frame, clicked, *SerilList)
drop.place(relx=0,rely=0.2)

openSerialPort = tk.Button(frame, text="Open Port", padx=10, pady=5, fg="white", bg="#263D42",command=openPort)
openSerialPort.place(relx=0.2,rely= 0.2)

closePort = tk.Button(frame, text="close port", padx=10, pady=5, fg="white", bg="#263D42",command=closeport)
closePort.place(relx=0.4,rely= 0.2)

HomeAll = tk.Button(frame, text="Home All", padx=10, pady=5, fg="white", bg="#263D42",command=HomeMotor)
HomeAll.place(relx=0.5,rely= 0.2)


zeroAll = tk.Button(frame, text="Zero All", padx=10, pady=5, fg="white", bg="#263D42",command=zeroAllmotor)
zeroAll.place(relx=0.7,rely= 0.2)

ShutAllMotor = tk.Button(frame, text="shut Motor", padx=10, pady=5, fg="white", bg="#263D42",command=shutMotor)
ShutAllMotor.place(relx=0.8,rely= 0.2)

button_auto = tk.Button(frame, text="auto", padx=10, pady=5, fg="white", bg="#263D42",command=change_auto_stat)
button_auto.place(relx=0.9,rely= 0.2)

mot1 = guiclass.guiclass(root,1,0.04,0.11)
mot1.bindkeys('D','A','d','a')
mot2 = guiclass.guiclass(root,2,0.28,0.11)
mot2.bindkeys('E','Q','e','q')
mot3 = guiclass.guiclass(root,3,0.52,0.11)
mot3.bindkeys('O','I','o','i')
mot4 = guiclass.guiclass(root,4,0.76,0.11)
mot4.bindkeys('W','S','w','s')
allmotor.append(mot1)
allmotor.append(mot2)
allmotor.append(mot3)
allmotor.append(mot4)




root.bind('!',mot1.HomeMotor)
root.bind('@',mot2.HomeMotor)
root.bind('#',mot3.HomeMotor)
root.bind('$',mot4.HomeMotor)

root.bind("<space>",startaction)
root.mainloop()
Step.stopCommunication()
