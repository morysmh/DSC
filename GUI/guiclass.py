import tkinter as tk
from tkinter import *
import Step
import re
import math
class guiclass():
    sizeframeX = 0.2
    sizeframeY = 0.5
    def __init__(self,tkroot: tk,motorNo,posx,posy,maxspeed = 150,lowspeed = 2000):
        self.position = 0
        self.comActive = 0
        self.root = tkroot
        self.posx = posx
        self.posy = posy
        self.isMoving = 0
        self.maxspeed = maxspeed
        self.lowspeed = lowspeed
        self.motorNo = motorNo 
        self.StringMotorPos = StringVar() 
        self.StringUserPos = StringVar() 
        self.speedString = StringVar() 
        self.stringReadPos = StringVar()
        self.frame = tk.Frame(self.root, bg="white")
        self.frame.place(relwidth=self.sizeframeX, relheight=self.sizeframeY,relx=self.posx, rely=self.posy)
        self.Helpframe = Label(self.frame,text="help frame",fg="black",font="Arial 12")
        self.Helpframe.place(relx=0.1,rely= .02)
        self.lowspeedKey = tk.Button(self.frame, text="Low Speed", padx=40, pady=10, fg="white", bg="#263D42",command=self.func_low)
        self.lowspeedKey.place(relx=0.1,rely= 0.2)
        self.isreachbutton = tk.Button(self.frame, text=" ", padx=5, pady=5, fg="white", bg="#FF0000")
        self.isreachbutton.place(relx=0.9,rely= 0.1)
        self.switchlowStat = 1
        self.speedHelp = Label(self.frame,text="Speed Input:",fg="black",font="Arial 12")
        self.speedHelp.place(relx=0.1,rely= .35)
        self.entry1 = tk.Entry(self.frame,textvariable=self.speedString, font=('calibre',14,'normal'),fg="BLACK")
        self.entry1.place(relx=0.1,rely=0.45)
        self.posHelp = Label(self.frame,text="Position Input:",fg="black",font="Arial 12")
        self.posHelp.place(relx=0.1,rely= .55)
        self.entryPos = tk.Entry(self.frame,textvariable=self.StringUserPos, font=('calibre',14,'normal'),fg="BLACK")
        self.entryPos.place(relx=0.1,rely=0.65)
        self.textMotor = Label(self.frame,textvariable=self.StringMotorPos,fg="black",font="Arial 20")
        self.textMotor.place(relx=0.1,rely= .8)
        self.textpos = Label(self.frame,textvariable=self.stringReadPos,fg="black",font="Arial 20")
        self.textpos.place(relx=0.1,rely= .9)                                                                  
        self.userposition = 0
        self.speed = 0
        self.negIsPossible = 0
        self.stringReadPos.set("0um")
        self.StringUserPos.set("0um")
        self.user_input_to_integer("0nm")
        self.setspeed(self.maxspeed)
        self.set_Position_Releative(0)
        self.isMotorActive = 0
        self.set_read_position(0)

    def setmotorPos(self,readpos):
        self.stringReadPos.set(readpos)

    def setspeed(self,speed):
        if speed < 10:
            speed = 10
        if speed > 50000:
            speed = 50000
        if self.speed == speed:
            return
        self.speed = speed
        self.speedString.set(self.speed)
        if self.comActive == 0:
            self.speed = 0
            return
        Step.stpeedmot(self.motorNo,self.speed)

    def func_low(self):
        self.switchlowStat = not self.switchlowStat
        if self.switchlowStat:
            self.setspeed(self.maxspeed)
            self.lowspeedKey.config(bg="#263D42")
        else:
            self.setspeed(self.lowspeed)
            self.lowspeedKey.config(bg="#81e381")

    def convertPosLinear(self,pos):
        neg = 0
        if pos < 0:
            neg = 1
            pos = pos * -1
        xx = "nm"
        if(pos > 1000):
            xx = "um"
            pos = pos / 1000
        if(pos > 1000):
            xx = "mm"
            pos = pos / 1000
        strval = str(pos) + xx
        if neg == 1:
            strval = '-' + str(pos) + xx
        return strval

    def set_read_position(self,readPos):
        self.read_pos = readPos
	
    def set_motorStat(self,status):
        self.isMoving = status & 0b10
        if(self.isMoving):
            self.isreachbutton.config(bg="#00FF00")
        else:
            self.isreachbutton.config(bg="#FF0000")
    
    def is_reachSetPos(self):
        return not self.isMoving
	
    def set_Position_Absoulute(self,pos):
        if (self.negIsPossible == 0) and (pos < 0):
            pos = 0
        self.position = int(pos)
        self.StringMotorPos.set(self.convertPosLinear(self.position))
        self.StringUserPos.set(self.convertPosLinear(self.position))

    def set_Position_Releative(self,position):
        self.position = position + self.position
        if (self.negIsPossible == 0) and (self.position < 0):
            self.position = 0
        self.StringMotorPos.set(self.convertPosLinear(self.position))
        self.StringUserPos.set(self.convertPosLinear(self.position))

    def pos_plus_min(self,events):
        self.set_Position_Releative(5000)

    def pos_neg_min(self,events):
        self.set_Position_Releative(-5000)

    def pos_large_plus(self,events):
        self.set_Position_Releative(1000000)

    def pos_large_neg(self,events):
        self.set_Position_Releative(-1000000)
    
    def can_Go_Negetive(self):
        self.negIsPossible = 1

    def user_input_releative(self,inputval):
        ri = 0
        rinput = re.sub(r'[^0-9.]','',inputval)
        try:
            ri = float(rinput)
        except:
            ri = float(re.match(r'^-*\d+',rinput).group())
        ri = round(ri,3)
        if re.search(r'[u]',inputval):
            ri = ri * 1000
        elif re.search(r'[n]',inputval):
            ri = ri 
        elif re.search(r'[m]',inputval):
            ri = ri * 1000000

        ri = int(round(ri,0))
        if re.search(r'[-]',inputval):
            ri = ri * (-1)
        self.set_Position_Releative(ri)
        return self.convertPosLinear(ri)

    def user_input_to_integer(self,inputval):
        ri = 0
        rinput = re.sub(r'[^0-9.]','',inputval)
        try:
            ri = float(rinput)
        except:
            ri = float(re.match(r'^-*\d+',rinput).group())
        ri = round(ri,3)
        if re.search(r'[u]',inputval):
            ri = ri * 1000
        elif re.search(r'[n]',inputval):
            ri = ri 
        elif re.search(r'[m]',inputval):
            ri = ri * 1000000
        ri = int(round(ri,0))
        self.set_Position_Absoulute(ri)
        return self.convertPosLinear(ri)


    def send_Position_To_Step(self):
        self.StringUserPos.set(self.user_input_to_integer(self.StringUserPos.get()))
        self.speedString.set(re.sub(r'[^0-9]','',self.speedString.get()))
        self.setspeed(int(self.speedString.get()))
        self.enable_motor()
        if self.comActive == 0:
            return
        Step.moveMotor(self.motorNo,self.position)

    def moveMotor(self):
        Step.startMoving(self.motorNo)

    def recoverMotor(self):
        Step.recoverMotor(self.motorNo)

    def enable_motor(self):
        if self.comActive == 0:
            return
        if self.isMotorActive == 1:
            return
        Step.EnableMotor(self.motorNo)
        self.isMotorActive = 1
    
    def shutmotor(self):
        if self.comActive == 0:
            return
        if self.isMotorActive == 0:
            return
        Step.disableMotor(self.motorNo)
        self.isMotorActive = 0
    
    
    def HomeMotor(self,event):
        if self.comActive == 0:
            return
        self.enable_motor()
        Step.homeMotor(self.motorNo)

    def bindkeys(self,posLarge,negLarge,posmin,negmin):
        helptxt = "Motor No " + str(self.motorNo) + "\nkeys FWD = " + str(posLarge) + str(posmin) + "\nkeys Backward = " + str(negLarge)+ str(negmin)
        self.Helpframe.configure(text=helptxt)
        self.root.bind("<" + posLarge + ">",self.pos_large_plus)
        self.root.bind("<" + negLarge + ">",self.pos_large_neg)
        self.root.bind("<" + posmin + ">",self.pos_plus_min)
        self.root.bind("<" + negmin + ">",self.pos_neg_min)
    
    def enable_comnmunicatin(self):
        self.comActive = 1
