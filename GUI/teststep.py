import Step
import time
motNO = 3
Step.openSerialPort("/dev/ttyACM1")
#Step.EnableMotor(motNO)
#Step.moveMotor(motNO,6000000)
#Step.startMoving(motNO)
#time.sleep(6)

Step.homeMotor(motNO)
Step.startMoving(motNO)

time.sleep(1)

Step.stopCommunication()
