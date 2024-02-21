import tkinter as tk
from tkinter import *
import os
import sys
import glob
import serial
import Step
import re
import time


_MaxSpeed1 = 1500
_MaxSpeed2 = 150
_MaxSpeed3 = 1500
_MaxSpeed4 = 150


motorcodes= [1,2,3,4]
motX = 0
motY = 0
motR = 0
motP = 0
motorShut = 0
switch_speed1 = 0
switch_speed2 = 0
switch_speed3 = 0
switch_speed4 = 0

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

def closeport():
    Step.stopCommunication()
def openPort():
    global _prev1,_prev2,_prev3,_prev4
    if(clicked.get().startswith("COM") == False):
        print("Not ON Windows")
        return
    Step.openSerialPort(clicked.get())
    shutMotor()
    Step.firstmotorRun()
    Step.stpeedmot(1,_prev1)
    Step.stpeedmot(2,_prev2)
    Step.stpeedmot(3,_prev3)
    Step.stpeedmot(4,_prev4)
    refresh_Txt_section()


def enableallmotor():
    global motorShut
    global motorcodes
    if motorShut == 0:
        return
    for i in motorcodes:
        Step.EnableMotor(i)
    motorShut = 0

def show():
    Label.config(text= clicked.get())

def shutMotor():
    global motorShut
    global motorcodes
    motorShut = 1
    for i in motorcodes:
        Step.disableMotor(i)

def HomeMotor():
    #Step.homeMotor(1)
    #Step.homeMotor(2)
    global motR
    motR = 10 * (10**6)
    motRchar.set(convertPosLinear(motR))

    Step.moveMotor(2,motR)
    Step.homeMotor(3)
    Step.homeMotor(4)
    
readstatValue = 0
def getPosition():
    global readstatValue
    if (readstatValue == 0):
        return
    root.after(700,getPosition)              
    for i in range(1,5):
        Step.sendCommand(motor=i,dest=Step.Ecom["C_Interface_Server"],\
        reg=Step.Ecom["C_command_Encoder_get_position"],val = Step.Ecom["C_Interface_ENCODER"])

def readmotorStatus():
    global readStat
    global readstatValue
    if readstatValue:
        readstatValue = 0
        readStat.config(bg="#263D42")
    else:
        readstatValue = 1
        readStat.config(bg="#81e381")
        getPosition()

def zeroAllmotor():
    Step.homeMotor(1)
    Step.homeMotor(2)
    Step.homeMotor(3)
    Step.homeMotor(4)

def func_low1():
    global switch_speed1
    global _prev1,_MaxSpeed1
    if switch_speed1:
        switch_speed1 = 0
        _prev1 = _MaxSpeed1
        lowspeed1.config(bg="#263D42")
    else:
        switch_speed1 = 1
        _prev1 = 2800
        lowspeed1.config(bg="#81e381")
    Step.stpeedmot(1,_prev1)
    _speedVar1.set(_prev1)

def func_low2():
    global switch_speed2
    global _prev2,_MaxSpeed2
    if switch_speed2:
        switch_speed2 = 0
        _prev2 = _MaxSpeed2
        lowspeed2.config(bg="#263D42")
    else:
        switch_speed2 = 1
        _prev2 = 1200
        lowspeed2.config(bg="#81e381")
    Step.stpeedmot(2,_prev2)
    _speedVar2.set(_prev2)
def func_low3():
    global switch_speed3
    global _prev3,_MaxSpeed3
    if switch_speed3:
        _prev3 = _MaxSpeed3
        switch_speed3 = 0
        lowspeed3.config(bg="#263D42")
    else:
        _prev3 = 2100
        switch_speed3 = 1
        lowspeed3.config(bg="#81e381")
    Step.stpeedmot(3,_prev3)
    _speedVar3.set(_prev3)

def func_low4():
    global switch_speed4
    global _prev4,_MaxSpeed4
    if switch_speed4:
        _prev4 = _MaxSpeed4
        switch_speed4 = 0
        lowspeed4.config(bg="#263D42")
    else:
        switch_speed4 = 1
        _prev4 = 2000
        lowspeed4.config(bg="#81e381")
    Step.stpeedmot(4,_prev4)
    _speedVar4.set(_prev4)

def setmotorPos(motor,inc_dec_val):
    global motP
    global motX
    global motY
    global motR
    if(motor == 'd'):
        motP = motP + inc_dec_val
        stepval = motP
    elif(motor == 'a'):
        motX = motX + inc_dec_val
        stepval = motX
    elif(motor == 'e'):
        motY = motY + inc_dec_val
        stepval = motY
    elif(motor == 'b'):
        motR = motR + inc_dec_val
        stepval = motR
    else:
        return

    #if(stepval < 0):
        #stepval = 0
    if(motY < 0):
        motY = 0
    if(motX < 0):
        motX = 0
    if(motR < 0):
        motR = 0
    #if(motP < 0):
        #motP = 0
    motPchar.set(convertPosLinear(motP))
    motRchar.set(convertPosLinear(motR))
    motXchar.set(convertPosLinear(motX))
    motYchar.set(convertPosLinear(motY))
_prev1 = _MaxSpeed1
_prev2 = _MaxSpeed2
_prev3 = _MaxSpeed3
_prev4 = _MaxSpeed4

def speedrangeCheck(_invar):
    if(_invar < 1):
        _invar = 1
    elif(_invar > 200000):
      _invar = 200000
    return _invar

def checkPrevspeed(_prev,_nextval,_mot):
    if _prev == int(_nextval.get()):
        return _prev
    _prev = speedrangeCheck(int(_nextval.get()))
    Step.stpeedmot(_mot, _prev)
    return _prev

def setspeedmotor():
    global _prev1,_prev2,_prev3,_prev4
    global _speedVar1,_speedVar2,_speedVar3,_speedVar4

    for i in (_speedVar1,_speedVar2,_speedVar3,_speedVar4):
        i.set(re.sub(r'[^0-9]','',i.get()))

    _prev1 = checkPrevspeed(_prev1, _speedVar1,1)
    _prev2 = checkPrevspeed(_prev2, _speedVar2,2)
    _prev3 = checkPrevspeed(_prev3, _speedVar3,3)
    _prev4 = checkPrevspeed(_prev4, _speedVar4,4)

def startaction(event):
    global motorcodes
    motor_gui = ['a','b','e','d']
    setspeedmotor()
    enableallmotor()
    for i in motor_gui:
        val = 0
        mot = 0
        if i == 'a':
            val = motX
            mot = 4
        if i == 'b':
            val = motR
            mot = 2
        if i == 'd':
            val = motP
            mot = 3
        if i == 'e':
            val = motY
            mot = 1
        Step.moveMotor(mot,val)

def actionHome(event):
    HomeMotor()

def home1(event):
    Step.homeMotor(1)
    #Step.moveMotor(1, 0)

def home2(event):
    Step.homeMotor(2)

def home3(event):
    Step.homeMotor(3)

def home4(event):
    Step.homeMotor(4)

def movemotPPos(event):
    setmotorPos('d',5000)

def movemotPNeg(event):
    setmotorPos('d',-5000)

def movemotPPos_100um(event):
    setmotorPos('d',100000)

def movemotPNeg_100um(event):
    setmotorPos('d',-100000)

def X_Pos5um(event):
    setmotorPos('a',10000)

def X_Pos100um(event):
    setmotorPos('a',1000000)

def X_Neg5um(event):
    setmotorPos('a',-10000)

def X_Neg100um(event):
    setmotorPos('a',-1000000)

def movemotRPos(event):
    setmotorPos('b',5000)

def movemotRNeg(event):
    setmotorPos('b',-5000)

def movemotRPos_100um(event):
    setmotorPos('b',100000)

def movemotRNeg_100um(event):
    setmotorPos('b',-100000)

def Y_Pos5um(event):
    setmotorPos('e',5000)

def Y_Pos100um(event):
    setmotorPos('e',1000000)

def Y_Neg5um(event):
    setmotorPos('e',-5000)

def Y_Neg100um(event):
    setmotorPos('e',-1000000)

def GuiMotor(_frame,_textHelp,_funcPTR,_motVar,_currpos):
    textMotor = Label(_frame,textvariable=_motVar,fg="black",font="Arial 20")
    textMotor.place(relx=0.1,rely= .7)
    textpos = Label(_frame,textvariable=_currpos,fg="black",font="Arial 20")
    textpos.place(relx=0.1,rely= .85)                                                                  
    Helpframe = Label(_frame,text=_textHelp,fg="black",font="Arial 12")
    Helpframe.place(relx=0.1,rely= .02)
    lowspeedKey = tk.Button(_frame, text="Low Speed", padx=60, pady=10, fg="white", bg="#263D42",command=_funcPTR)
    lowspeedKey.place(relx=0.1,rely= 0.25)
    return lowspeedKey

def GuiSpeedInput(_frame,_var):
    entry1 = tk.Entry(_frame,textvariable=_var, font=('calibre',14,'normal'),fg="BLACK")
    entry1.place(relx=0.1,rely=0.5)
    return entry1

def printvarcurrposi(i_val):
    i_val = i_val.split(" ")
    try:
        if(int(i_val[2]) != 19):
            return
    except:
        return
    if(int(i_val[1]) == 1):
        motYposcurrent.set(convertPosLinear(int(i_val[3])))
    if(int(i_val[1]) == 2):
        motRposcurrent.set(convertPosLinear(int(i_val[3])))
    if(int(i_val[1]) == 3):
        motPposcurrent.set(convertPosLinear(int(i_val[3])))
    if(int(i_val[1]) == 4):
        motXposcurrent.set(convertPosLinear(int(i_val[3])))

def print_current_Pos( i_dat):
    if(i_dat == ""):
        return
    s_a = i_dat.split("\n")
    for i in s_a:
        printvarcurrposi(i)

def refresh_Txt_section():
    global txt_output
    global root
    a = Step.readcomm()
    print_current_Pos(a)
    txt_output.configure(state='normal')
    txt_output.insert(END,a)
    txt_output.configure(state='disable')
    root.after(700,refresh_Txt_section)              
root = tk.Tk()

motXchar = StringVar() 
motYchar = StringVar() 
motRchar = StringVar() 
motPchar = StringVar() 

motXposcurrent = StringVar() 
motYposcurrent = StringVar() 
motRposcurrent = StringVar() 
motPposcurrent = StringVar()

_speedVar1 = tk.StringVar()
_speedVar2 = tk.StringVar()
_speedVar3 = tk.StringVar()
_speedVar4 = tk.StringVar()


motXchar.set(str(0)) 
motYchar.set(str(0)) 
motRchar.set(str(0)) 
motPchar.set(str(0)) 

motXposcurrent.set(str(0))
motYposcurrent.set(str(0))
motRposcurrent.set(str(0))
motPposcurrent.set(str(0))
                          
SerilList = serial_ports()
basewidthGUI = 1400
canvas = tk.Canvas(root, height=800, width=basewidthGUI, bg="#253D42")
canvas.pack()

frame = tk.Frame(root, bg="white")
frame.place(width=basewidthGUI-basewidthGUI*.1, height=50,relx=0.05, rely=0.01)

yFrame = tk.Frame(root, bg="white")
yFrame.place(relwidth=.2, relheight=0.5,relx=0.04, rely=0.11)

pFrame = tk.Frame(root, bg="white")
pFrame.place(relwidth=.2, relheight=0.5,relx=0.28, rely=0.11)

rFrame = tk.Frame(root, bg="white")
rFrame.place(relwidth=.2, relheight=0.5,relx=0.52, rely=0.11)

xFrame = tk.Frame(root, bg="white")
xFrame.place(relwidth=.2, relheight=0.5,relx=0.76, rely=0.11)

txt_Frame = tk.Frame(root, bg="white")
txt_Frame.place(relwidth=.9, relheight=0.2,relx=0.05, rely=0.65)

clicked = StringVar()
if(SerilList):
    clicked.set(SerilList[0])
else:
    clicked.set("Select Port")

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

readStat = tk.Button(frame, text="read Stat", padx=10, pady=5, fg="white", bg="#263D42",command=readmotorStatus)
readStat.place(relx=0.6,rely= 0.2)
ShutAllMotor = tk.Button(frame, text="shut Motor", padx=10, pady=5, fg="white", bg="#263D42",command=shutMotor)
ShutAllMotor.place(relx=0.8,rely= 0.2)



lowspeed1 = GuiMotor(yFrame, "Motor 1 key: aA dD", func_low1, motYchar, motYposcurrent)
lowspeed2 = GuiMotor(pFrame, "Motor 2 key: eE qQ", func_low2, motRchar, motRposcurrent)
lowspeed3 = GuiMotor(rFrame, "Motor 3 key: oO iI", func_low3, motPchar, motPposcurrent)
lowspeed4 = GuiMotor(xFrame, "Motor 4 key: wW sS", func_low4, motXchar, motXposcurrent)



GuiSpeedInput(yFrame,_speedVar1)
GuiSpeedInput(rFrame,_speedVar3)
GuiSpeedInput(pFrame,_speedVar2)
GuiSpeedInput(xFrame,_speedVar4)

_speedVar1.set(_prev1)
_speedVar2.set(_prev2)
_speedVar3.set(_prev3)
_speedVar4.set(_prev4)


txt_output = Text(txt_Frame,state= 'disable')
txt_output.pack(pady=30)

root.bind("<w>",X_Pos5um)
root.bind("<s>",X_Neg5um)

root.bind("<W>",X_Pos100um)
root.bind("<S>",X_Neg100um)

root.bind("<d>",Y_Pos5um)
root.bind("<a>",Y_Neg5um)

root.bind("<D>",Y_Pos100um)
root.bind("<A>",Y_Neg100um)

root.bind("<e>",movemotRPos)
root.bind("<q>",movemotRNeg)

root.bind("<E>",movemotRPos_100um)
root.bind("<Q>",movemotRNeg_100um)

root.bind("<o>",movemotPPos)
root.bind("<i>",movemotPNeg)
root.bind("<O>",movemotPPos_100um)
root.bind("<I>",movemotPNeg_100um)
root.bind("<space>",startaction)

root.bind(')',actionHome)

root.bind('!',home1)
root.bind('@',home2)
root.bind('#',home3)
root.bind('$',home4)

root.mainloop()
Step.stopCommunication()