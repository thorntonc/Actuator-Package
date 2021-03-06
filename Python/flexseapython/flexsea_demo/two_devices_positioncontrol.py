import os, sys
from time import sleep

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

def printDevice(actPackState):
	print('State time: ', actPackState.timestamp)
	print('Accel X: ', actPackState.accelx, ', Accel Y: ', actPackState.accely, ' Accel Z: ', actPackState.accelz)
	print('Gyro X: ', actPackState.gyrox, ', Gyro Y: ', actPackState.gyroy, ' Gyro Z: ', actPackState.gyroz)
	print('Motor angle: ', actPackState.encoderAngle, ', Motor voltage: ', actPackState.motorVoltage, flush=True)



def fxTwoDevicePositionControl(port0, port1, baudRate):

	devId0 = fxOpen(port0, baudRate, 0)
	devId1 = fxOpen(port1, baudRate, 0)

	fxStartStreaming(devId0, 200, True)
	fxStartStreaming(devId1, 200, True)

	sleep(0.2)

	actPackState0 = fxReadDevice(devId0)
	actPackState1 = fxReadDevice(devId1)

	initialAngle0 = actPackState0.encoderAngle
	initialAngle1 = actPackState1.encoderAngle

	fxSetGains(devId0, 50, 3, 0, 0, 0)
	fxSetGains(devId1, 50, 3, 0, 0, 0)
	
	fxSendMotorCommand(devId0, FxPosition, initialAngle0)
	fxSendMotorCommand(devId1, FxPosition, initialAngle1)

	try:
		while(True):
			sleep(0.2)
			os.system('cls')
			preamble = "Holding position, two devices: "
			print(preamble)
	
			actPackState0 = fxReadDevice(devId0)
			actPackState1 = fxReadDevice(devId1)
			
			printDevice(actPackState0)
			printDevice(actPackState1)

	except:
		pass

	print('Turning off position control...')
	fxClose(devId0)
	fxClose(devId1)

if __name__ == '__main__':
	baudRate = sys.argv[1]
	ports = sys.argv[2:4]
	try:
		fxTwoDevicePositionControl(ports[0], ports[1], baudRate)
	except Exception as e:
		print("broke: " + str(e))
