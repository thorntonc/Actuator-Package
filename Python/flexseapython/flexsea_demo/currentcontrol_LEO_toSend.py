import os, sys
from time import sleep
from time import time, strftime, process_time,perf_counter
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import correlate

sys.path.append(r"../..") # set the directory based on your computer  Use relative paths PLZ


from flexseapython.pyFlexsea import *
from flexseapython.pyFlexsea_def import *
from flexseapython.fxUtil import *


#conversion factors
degToCount = 45.5111 
countToDeg = 1/degToCount

labels = ["State time", 											\
"accel x", "accel y", "accel z", "gyro x", "gyro y", "gyro z", 		\
"encoder angle", "motor voltage", "motor current","ankle angle", "ankle velocity"									\
]

varsToStream = [ 							\
	FX_STATETIME, 							\
	FX_ACCELX, FX_ACCELY, FX_ACCELZ, 		\
	FX_GYROX,  FX_GYROY,  FX_GYROZ,			\
	FX_ENC_ANG,								\
	FX_MOT_VOLT,							\
	FX_MOT_CURR,							\
	FX_ANKLE_ANG,							\
	FX_ANKLE_ANG_VEL						\

]

def fxCurrentControl(devId):
	holdCurrent = 300 * (np.sin(2 * np.pi * 1 * ((0))))
	fxSetStreamVariables(devId, varsToStream)
	streamSuccess = fxStartStreaming(devId, 500, False, 0)

	print('Setting controller to current...')
	setControlMode(devId, CTRL_CURRENT)
	setGains(devId, 100, 20, 0, 0)
	

	# Init data arrays
	timeVec = np.array([])
	currentVec_des = np.array([])
	currentVec_act = np.array([])

	# track reads and nones
	validReads = 0
	noneReads = 0

	interval = 10*60 #seconds

	# Perf counter accounts for sleeps and is system wide (See python doc)
	startTime = perf_counter()
	lastCheck = startTime
	currentTime = perf_counter()
	time_step = 1/500; #seconds

	print('Entering While loop')

	while((currentTime - startTime) < interval):
		#print("Holding Current: {} mA...".format(holdCurrent))
		currentTime = perf_counter()
		timeSec = currentTime - lastCheck
		timeLapsed = currentTime - startTime
		
		if (timeSec) > time_step:
			lastCheck = currentTime
			deviceValue = (fxReadDevice(devId, [FX_ENC_ANG])[0])
			if deviceValue is not None: 
				validReads += 1
				holdCurrent = 300 * (np.sin(2 * np.pi * 1 * ((timeLapsed))))
				setMotorCurrent(devId, holdCurrent) # Start the current, holdCurrent is in mA 
				currentVec_act = np.append(currentVec_act, deviceValue )
				timeVec = np.append(timeVec, timeLapsed) 
				currentVec_des = np.append(currentVec_des, holdCurrent)
			else:
				noneReads += 1
				print("Data is not available")
				holdCurrent = 0

	print("Exiting Procedure\n")
	print("Time: {}".format(currentTime - startTime))
	print("Valid reads:  {}".format(validReads))
	print("None reads:  {}".format(noneReads))
	print("Average execution frequency: {}".format(float(i)/(currentTime - startTime))  )





	print('Turning off current control...')

	print("Time: {}".format(currentTime - startTime))
	print("i:  {}".format(i))
	print("Average execution frequency: {}".format(float(i)/(currentTime - startTime))  )

	# ramp down first
	n = 50
	for i in range(0, n):
		setMotorCurrent(devId, holdCurrent * (n-i)/n)
		sleep(0.04)

	# wait for motor to spin down

	setMotorCurrent(devId, 0)

	setControlMode(devId, CTRL_NONE)
	fxStopStreaming(devId)


	
	dataMatrix = np.vstack((currentVec_des ,currentVec_act))
	#print(dataMatrix[0,:].shape)
	#correlatedSignal = correlate(dataMatrix[0,:],dataMatrix[1,:])
	#lag = (len(correlatedSignal)/2) - np.argmax(correlatedSignal)
	#c_sig = np.roll(b_sig, shift=int(np.ceil(lag)))
	#print("Lag is ", lag, " ticks")
	dataMatrix = dataMatrix.transpose()
	plt.plot(dataMatrix)
	print(5/i)
	plt.xlabel("Time in ticks")
	plt.ylabel("Current in mA")
	
	filename = 'currentControl_Leo_toSend' + strftime("%Y%m%d-%H%M%S") + '.csv'
	plt.savefig('delay_times_' + strftime("%Y%m%d-%H%M%S") + '.png')
	np.savetxt(filename, dataMatrix, fmt='%.5f', delimiter=',')

def loadAndGetDevice(filename, numDevices=None):
	loadSuccess = loadFlexsea()
	if(not loadSuccess):
#		print("Library load failed... quitting")
		sys.exit('\nload FlexSEA failed')

	isUnix = os.name != 'nt'
	if(isUnix and os.geteuid() != 0):
		sys.exit('\nRoot privileges needed for running this script')

	if(numDevices != None):
		portList = []
		with open(filename, 'r') as f:
			portList = [ line.strip() for line in f if ( line.strip() != '' ) ]
	else:
		portList = filename
		numDevices = len(portList)

	n = len(portList)
	if(n > FX_NUM_PORTS):
		n = FX_NUM_PORTS

	for i in range(0, n):
		fxOpen(portList[i], i)
		sleep(0.2)
	
	waiting = True
	waited = 0
	while(waiting and waited < 5):
		sleep(0.2)
		waited = waited + 0.2

		waiting = False
		for i in range(0, n):
			if(not fxIsOpen(i)):
				print("Waiting for port {} to be open".format(i))
				waiting = True

	if(waiting):
		print("Couldn't connect...")
		sys.exit(1)
	
	devIds = fxGetDeviceIds()

	while(len(devIds) < numDevices):
		sleep(0.1)
		devIds = fxGetDeviceIds()
	
	return devIds

if __name__ == '__main__':
	ports = sys.argv[1:2]
	devId = loadAndGetDevice(ports)[0]
	fxCurrentControl(devId)	

