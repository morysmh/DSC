import serial
import crc
import time
COM = 'COM11'
baud = 115200
firstrun = 0
ser = []
Ecom = dict(
    C_Interface_NULL = 0,
    C_Interface_STEP_CTRL = 1,
    C_Interface_STEP_Pulse = 2,
    C_Interface_CAN = 3,
    C_Interface_INPUTSTOP = 4,
    C_Interface_ENCODER = 5,
    C_Interface_Server = 6,
    C_Command_NULL = 0,
    C_command_STEP_Pulse_set_STOP = 1,
    C_command_STEP_Pulse_set_step = 2,
    C_command_STEP_Pulse_set_duration = 3,
    C_command_STEP_Pulse_set_acc = 4,
    C_command_STEP_Pulse_set_interval_acc = 5,
    C_command_STEP_Pulse_set_MAX = 6,
    C_command_STEP_Pulse_set_min = 7,
    C_command_STEP_Pulse_get_remain = 8,
    C_command_STEP_CTRL_set_pulse_per_100mm = 9,
    C_command_STEP_CTRL_set_microstep = 10,
    C_command_STEP_CTRL_set_direction_default = 11,
    C_command_STEP_CTRL_set_endstop = 12,
    C_command_STEP_CTRL_set_topstop = 13,
    C_command_STEP_CTRL_get_current_position = 14,
    C_command_STEP_CTRL_DisableMotor = 15,
    C_command_STEP_CTRL_EnableMotor = 16,
    C_command_STEP_CTRL_set_TO_GO_nm = 17,
    C_command_STEP_CTRL_set_encoder_feedback = 18,
    C_command_Encoder_get_position = 19,
    C_command_Encoder_set_zero = 20,
    C_command_Encoder_set_nm_per_pulse = 21,
    C_command_Encoder_get_nm_pp = 22,
    C_command_Encoder_set_direction = 23,
    C_command_STEP_CTRL_go_home = 24,
    C_command_Keyd_default_min = 27,
    C_command_Keyd_default_MAX = 28,
    C_command_STEP_CTRL_Home_Direction = 29)
def openSerialPort(comnumber = COM):
    global ser
    ser = serial.Serial(port=comnumber,baudrate=baud)
    firstmotorRun()

def firstmotorRun():
    global firstrun
    if firstrun == 1:
        return
    print("First Time Runned")
    disableALL()
    sendCommand(motor=4,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_min"],val = 10)
    sendCommand(motor=4,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_MAX"],val = 50000)

    sendCommand(motor=4,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_encoder_feedback"],val = 1)

    sendCommand(motor=4,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_direction_default"],val = 0)
                
    sendCommand(motor=4,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_direction"],val = 1)

    sendCommand(motor=4,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_nm_per_pulse"],val = 5000)
    sendCommand(motor=4,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_pulse_per_100mm"],val = 1000000000)

    sendCommand(motor=4,dest=Ecom["C_Interface_INPUTSTOP"],\
                reg=Ecom["C_command_Keyd_default_MAX"],val = 0)

    sendCommand(motor=3,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_min"],val = 100)
    sendCommand(motor=3,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_MAX"],val = 50000)
    sendCommand(motor=3,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_encoder_feedback"],val = 1)

    sendCommand(motor=3,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_direction_default"],val = 0)
                
    sendCommand(motor=3,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_direction"],val = 1)

    sendCommand(motor=3,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_nm_per_pulse"],val = 500)

    sendCommand(motor=3,dest=Ecom["C_Interface_INPUTSTOP"],\
                reg=Ecom["C_command_Keyd_default_min"],val = 0)

    #sendCommand(motor=3,dest=Ecom["C_Interface_STEP_CTRL"],\
    #            reg=Ecom["C_command_STEP_CTRL_Home_Direction"],val = 1)

    sendCommand(motor=2,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_min"],val = 10)
    sendCommand(motor=2,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_MAX"],val = 50000)
    sendCommand(motor=2,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_encoder_feedback"],val = 1)

    sendCommand(motor=2,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_direction_default"],val = 0)
    
    sendCommand(motor=2,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_pulse_per_100mm"],val = 1000000000)
                
    sendCommand(motor=2,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_direction"],val = 1)

    sendCommand(motor=2,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_nm_per_pulse"],val = 5000)

    sendCommand(motor=1,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_min"],val = 10)
    sendCommand(motor=1,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_MAX"],val = 50000)

    sendCommand(motor=1,dest=Ecom["C_Interface_INPUTSTOP"],\
                reg=Ecom["C_command_Keyd_default_min"],val = 0)

    sendCommand(motor=1,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_nm_per_pulse"],val = 5000)
    sendCommand(motor=1,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_encoder_feedback"],val = 1)
    sendCommand(motor=1,dest=Ecom["C_Interface_ENCODER"],\
                reg=Ecom["C_command_Encoder_set_direction"],val = 1)

    firstrun = 1

def nmPefFullStep(mot,nanoMeter):
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_pulse_per_100mm"],val = nanoMeter)


def rs485_send(data):
    global ser
    if(not ser):
        return
    ser.write(bytes(data))
    print(data)
    time.sleep(.1)

def sendCommand(motor,dest,reg,val):
    b = [(motor & 0xFF),((reg>>8) & 0xFF),(reg & 0xFF),(dest & 0x0F),((val>>32) & 0xFF),((val>>24) & 0xFF),((val>>16) & 0xFF),((val>>8) & 0xFF),(val & 0xFF)]
    rs485_send(b)

def moveMotor(mot,nanoMeter):
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_set_TO_GO_nm"],val = nanoMeter)

def speedMotor(mot,speed):
    #speed = 5000 - speed
    if speed < 10:
        speed = 10
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_Pulse"],\
                reg=Ecom["C_command_STEP_Pulse_set_duration"],val = speed)

def homeMotor(mot):
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_go_home"],val = 1)
    
def EnableMotor(mot):
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_EnableMotor"],val = 0)
def disableMotor(mot):
    sendCommand(motor=mot,dest=Ecom["C_Interface_STEP_CTRL"],\
                reg=Ecom["C_command_STEP_CTRL_DisableMotor"],val = 0)

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
