import time
import spidev
import math
import argparse 
import sys
import navio.util
#import AccelGyroMag
import navio.mpu9250 # For magnetometer calibration only
import madgwickahrs.madgwickahrs as attitude
import navio.pwm
import navio.adc
import navio.leds
#import GPS
import numpy as np
import navio.ms5611
import os
import navio.rcinput
from transforms3d.euler import quat2euler
import serial
import struct
#port = serial.Serial("/dev/ttyUSB0",baudrate=57600, timeout=3.0)
# Getting next available log file number and creating header#
gg = 0
while os.path.exists("Log_Files/datalog%s.csv" % gg):
	gg+=1
header_string = "Time, Altitude, Yaw, Pitch, Roll, Ax, Ay, Az, Pitch_rate, Roll_rate,Yaw_rate, Magx, Magy, Magz"
fh = open("Log_Files/datalog%s.csv" % gg,"a")
fh.write(header_string)
fh.close()


print "Initializing ADC"
navio.util.check_apm()
adc = navio.adc.ADC()
analog = [0] * adc.channel_count

print "Initializing Sensors"
imu = navio.mpu9250.MPU9250()
imu.initialize()
rcin = navio.rcinput.RCInput()
AHRS_data = attitude.AHRS(0.01) #100Hz sensor attitude estimation FIXED 
## GPS Disabled
#GPS = GPS.U_blox()
#for ind in range(0, 10):
#	GPS.enable_posllh()
led = navio.leds.Led()
baro = navio.ms5611.MS5611()
baro.initialize()
time.sleep(0.25)
baro.refreshPressure()
time.sleep(0.01) # Waiting for pressure data ready 10ms
baro.readPressure()
baro.refreshTemperature()
time.sleep(0.01) # Waiting for temperature data ready 10ms
baro.readTemperature()
baro.calculatePressureAndTemperature()
ground_alt = 0
if baro.PRES < 1013: # small check in case barometer pressure is invalid
	ground_alt = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)

led.setColor('Red')
time.sleep(1)


if imu.testConnection():
    print "Connection established: True"
else:
    sys.exit("Connection established: False")
    
accels, rates, m9m = imu.getMotion9()
if m9m[0] == 0:
	print "WARNING: Mag reading zeros, try rebooting Navio"
	led.setColor('Magenta')

mag_calibration_flag = False # True means run calibration, 
							 # otherwise use data in 
							 # mag_calibration_constants.txt

 

# Run Magnetometer calibration if flag is TRUE
if mag_calibration_flag == True:
	led.setColor('Yellow')
	imu.mag_calibrate()

# Configure servo output
motor_front_pin = 0
motor_back_pin = 1
motor_left_pin = 2
motor_right_pin = 3
motor_front_pwm = navio.pwm.PWM(motor_front_pin)
motor_back_pwm = navio.pwm.PWM(motor_back_pin)
motor_left_pwm = navio.pwm.PWM(motor_left_pin)
motor_right_pwm = navio.pwm.PWM(motor_right_pin)
motor_front_pwm.set_period(200)
motor_back_pwm.set_period(200)
motor_left_pwm.set_period(200)
motor_right_pwm.set_period(200)

# Cycling the pwm commands to initialize the devices
motor_front_pwm.set_duty_cycle(1.000)
motor_back_pwm.set_duty_cycle(1.000)
motor_left_pwm.set_duty_cycle(1.000)
motor_right_pwm.set_duty_cycle(1.000)
time.sleep(0.5)
motor_front_pwm.set_duty_cycle(2.000)
motor_back_pwm.set_duty_cycle(2.000)
motor_left_pwm.set_duty_cycle(2.000)
motor_right_pwm.set_duty_cycle(2.000)
time.sleep(0.5)
motor_front_pwm.set_duty_cycle(1.000)
motor_back_pwm.set_duty_cycle(1.000)
motor_left_pwm.set_duty_cycle(1.000)
motor_right_pwm.set_duty_cycle(1.000)
time.sleep(0.5)

# Timers to maintain constant cycle times 
timein = time.time()
timeg = time.time() - timein
prev_time = timeg*1000.0
timer_1hz = prev_time
timer_10hz = prev_time
timer_25hz = prev_time
timer_50hz = prev_time
timer_100hz = prev_time
baro_timer = 0
# Declaring variables for use in main loop
rc_data = rcin.read_all()
motor_front = 1.000
motor_back = 1.000
motor_left = 1.000
motor_right = 1.000
gyro_2 = 0
gyro_1 = 0
gyro_0 = 0
cur_time = 0
roll_angle_gyro = 0
gyro_2p = 0
gyro_1p = 0
gyro_0p = 0
pitch_angle_gyro = 0
ac_AHRS = [0,0,0]
gy_AHRS = [0,0,0]
mag_AHRS = [0,0,0]
xpos = 0;ypos = 0;zpos = 0
wn = 4; damp = 1;kd = 2*damp*wn; kp = wn*wn; # controller gains
c = -0.15;b = 0.15 # motor constants
yawd = 0; coryaw = 0;yawddprev = 0;angleyaw = 0;n = 0;yaw1cmd = 0;yaw2cmd = 0;yaw3cmd = 0;yaw4cmd = 0


#------------------------------------------------#
###       Declare global variables here        ###

print "Starting main loop: here we go!"
while True:
	current_time = timeg*1000.0
	if m9m[0] == 0:
		led.setColor('Magenta')
	else:
		led.setColor('Green')
		
		
	if (current_time - timer_100hz) >=10.0: # 10 ms = 100Hz
		#### IMU/Attitude and GPS estimation: DO NOT TOUCH ####
		for i in range (0, adc.channel_count):
			analog[i] = adc.read(i)*0.001
		accels, rates, m9m = imu.getMotion9()
		ac_AHRS[0] = -accels[1]
		ac_AHRS[1] = -accels[0]
		ac_AHRS[2] = accels[2]
		gy_AHRS[0] = rates[1]
		gy_AHRS[1] = rates[0]
		gy_AHRS[2] = -rates[2]
		AHRS_data.update_imu(gy_AHRS, ac_AHRS)
		roll,pitch,yaw = quat2euler(AHRS_data.quaternion,axes='rxyz')
		baro_timer = baro_timer + 1
		if (baro_timer == 1): baro.refreshPressure()
		elif (baro_timer == 2): baro.readPressure()
		elif (baro_timer == 3): baro.refreshTemperature()
		elif (baro_timer == 4): baro.readTemperature()
		elif (baro_timer == 5):
			baro.calculatePressureAndTemperature()
			baro_timer = 0
			#print baro.PRES
			if baro.PRES < 1013: # Only update if barometer is valid
				#alts = 44330.77*(1-(baro.PRES*100/101326)**0.1902632)
				alts = 0
				current_alt = alts - ground_alt
		
		#buffer = GPS.bus.xfer2([100])
		## GPS is disabled ##
		#for byt in buffer:
		#	GPS.scan_ubx(byt)
		#	if(GPS.mess_queue.empty() != True):
		#		GPS_data = GPS.parse_ubx()
		#### End DO NOT TOUCH ####
		
		#----------------------------------------------------#
		#### 			BEGIN STUDENT SECTION			 ####
		
		
		# This section runs at 100Hz. You can add things for executation
		# at other speeds. See the 1Hz loop for display examples. SD 
		# logging occurs at 10Hz. See the 10Hz if statement for details
		
		# Output commands
		# 		motor_front, motor_back, motor_left, motor_right are the 4 motor commands
		# 		motor range is from 1.000 to 2.000 (1.000 is 0% power)
		''' serial read/ position'''
		'''port = open('/dev/ttyUSB0', 'rb')
		data = np.fromfile(port,np.uint16,1)
		if data==36:
			data = np.fromfile(port,np.uint16,1)
			if data==36:
				data = np.fromfile(port,np.uint16,7)
				#print data
				if len(data)>6:
					if data[6] == 35:
						port.flush()
						xpos = np.double(struct.unpack("H",data[0:2]))-32768
						xpos = xpos*.01
						ypos = np.double(struct.unpack("H",data[2:4]))-32768
						ypos = xpos*.01
						zpos = np.double(struct.unpack("H",data[4:6]))-32768
						zpos = xpos*.01
						port.close'''
		''' yaw angle calculation'''
		if angleyaw < 0.523 and yaw > 5.756:
			n = n-1
		elif yaw < 0.523 and  angleyaw> 5.756:
			n = n+1
		coryaw = yaw + n*6.28
		''' Main code '''
		if float(rc_data[4])<1300:
			motor_front = 1.000
			motor_back = 1.000
			motor_left = 1.000
			motor_right = 1.000
			yawd = coryaw
		roll1cmd = (kd*(-1*float(rates[1]))+kp*(5*(float(rc_data[0])/1000.0-1.513)-roll)+1.3*float(rates[1]))/b
		roll2cmd = (kd*(-1*float(rates[1]))+kp*(5*(float(rc_data[0])/1000.0-1.513)-roll)+1.3*float(rates[1]))/c
		yawd = ((float(rc_data[3])/1000.0 - 1.513) + yawddprev)*.01/2.0 + coryaw
		#yawd = (float(rc_data[3])/1000.0 - 1.513)*.01 + coryaw
		yawddprev = float(rc_data[3])/1000.0 - 1.513
		yaw1cmd = ((4*(float(rc_data[3])/1000.0 - 1.513 - rates[2]) + 4*(yawd - coryaw) + 0.13*rates[2])/0.009)
		yaw2cmd = ((4*(float(rc_data[3])/1000.0 - 1.513 - rates[2]) + 4*(yawd - coryaw) + 0.13*rates[2])/0.009)
		yaw3cmd = ((4*(float(rc_data[3])/1000.0 - 1.513 - rates[2]) + 4*(yawd - coryaw) + 0.13*rates[2])/-0.009)
		yaw4cmd = ((4*(float(rc_data[3])/1000.0 - 1.513 - rates[2]) + 4*(yawd - coryaw) + 0.13*rates[2])/-0.009)
		pitch1cmd = (kd*(-1*float(rates[0]))+kp*(5*(float(rc_data[1])/1000.0-1.513)-pitch)+1.3*float(rates[0]))/-0.15
		pitch2cmd = (kd*(-1*float(rates[0]))+kp*(5*(float(rc_data[1])/1000.0-1.513)-pitch)+1.3*float(rates[0]))/0.15
		throtcmd = 0.001138*float(rc_data[2])-0.2323
		timer = timeg
		sinr1=(.37*math.sin(1.5713+2*math.pi*0.2*timer) + .37*math.sin(4.5717+2*0.6*math.pi*timer) + .37*math.sin(1.2140+2*1.0*math.pi*timer) + .37*math.sin(1.0478+2*1.4*math.pi*timer) + .37*math.sin(3.9204+2*math.pi*1.8*timer) + .37*math.sin(4.0099+2*2.2*math.pi*timer) + .37*math.sin(3.4966+2*2.6*math.pi*timer))
		sinr2=(.37*math.sin(1.6146+2*math.pi*0.3*timer) + .37*math.sin(4.6867+2*0.7*math.pi*timer) + .37*math.sin(1.2267+2*1.1*math.pi*timer) + .37*math.sin(1.0671+2*1.5*math.pi*timer) + .37*math.sin(3.9664+2*math.pi*1.9*timer) + .37*math.sin(3.8699+2*2.3*math.pi*timer) + .37*math.sin(3.5712+2*2.7*math.pi*timer))
		sinr3=(.37*math.sin(1.9535+2*math.pi*0.4*timer) + .37*math.sin(5.2646+2*0.8*math.pi*timer) + .37*math.sin(2.0651+2*1.2*math.pi*timer) + .37*math.sin(2.4636+2*1.6*math.pi*timer) + .37*math.sin(5.6716+2*math.pi*2.0*timer) + .37*math.sin(5.7265+2*2.4*math.pi*timer) + .37*math.sin(5.7810+2*2.8*math.pi*timer))
		sinr4=(.37*math.sin(3.2771+2*math.pi*0.5*timer) + .37*math.sin(1.3417+2*0.9*math.pi*timer) + .37*math.sin(5.5561+2*1.3*math.pi*timer) + .37*math.sin(0.5030+2*1.7*math.pi*timer) + .37*math.sin(4.7331+2*math.pi*2.1*timer) + .37*math.sin(5.9415+2*2.5*math.pi*timer) + .37*math.sin(0.7460+2*2.9*math.pi*timer))
		if float(rc_data[4])>1500:
			motor_right = throtcmd+(roll2cmd+yaw1cmd)/1000.0
			motor_left = throtcmd+(roll1cmd+yaw2cmd)/1000.0
			motor_back = throtcmd+(pitch1cmd+yaw3cmd)/1000.0
			motor_front = throtcmd+(pitch2cmd+yaw4cmd)/1000.0
		angleyaw = yaw
		print yaw1cmd,roll1cmd,pitch1cmd,throtcmd
		#print roll1cmd
		#print rates
		#print motor_right,motor_left
		#print rc_data
		#print "Angles:", "{:+3.2f}".format(roll*57.32), "{:+3.2f}".format(pitch*57.32), "{:+3.2f}".format(yaw*57.32)
		# R/C Input 
		# 		rc_data variable stores all RC inputs (range is [0]-[5])
		#		each rc_data channel varies between 1000 and 2000
		
		# Sensor data
		#		yaw, roll, and pitch contain attitude info (float)
		#		rates[0 to 2] stores the three angular velocity components (rad/s)
		#		accels[0 to 2] stores the three linear accelerations
		#		analog[4] and [5] are the two analog inputs (value is in Volts)
		#		current_alt contains the current altitude (relatively to 
		#		start up) in meters (from the barometer)
		
		
		####		 END STUDENT SECTION				####
		#---------------------------------------------------#
		
		#port.close
		motor_front_pwm.set_duty_cycle(motor_front)
		motor_back_pwm.set_duty_cycle(motor_back)
		motor_left_pwm.set_duty_cycle(motor_left)
		motor_right_pwm.set_duty_cycle(motor_right)
		timer_100hz = current_time # reset timer flag
		# end of 100Hz section
	
	if (current_time - timer_50hz) >= 20.0:
		
		
		
		
		timer_50hz = current_time
		# End of 50Hz section
	
	if (current_time - timer_25hz) >= 40.0:
		
		
		
		
		timer_25hz = current_time
		# End of 25Hz section
		
	if (current_time - timer_10hz) >= 100.0:
		# RC Controller INPUT #
		rc_data = rcin.read_all()
		
		### Data logging feature ###
		# GPS is disabled, tab in fh and below to re-enable
		#try:
		#	GPS_data
		#except NameError:
		#	GPS_data = None
		#if GPS_data is not None:
		fh = open("Log_Files/datalog%s.csv" % gg,"a")
		#log_data = np.array([time.clock(), GPS_data.lat/10000000.0, GPS_data.lon/10000000.0,
		#### 			LOGGING 				####
		# This is the data to be logged. The header (text at top of file) is edited at the top
		# of the program. Add/subtract variables as needed.
		log_data = np.array([timeg,current_alt, yaw, pitch, roll, 
					accels[0], accels[1], accels[2], rates[0], rates[1], rates[2],
					m9m[0], m9m[1], m9m[2] ])
		np.savetxt(fh, log_data.reshape(1,log_data.shape[0]), delimiter=',', fmt='%.6f')
		
		fh.close()
		####			END LOGGING				####
		
		timer_10hz = current_time
		# End of 10Hz section
	
	if (current_time - timer_1hz) >= 1000.0:
		# Customizable display message #
		#print "Angles:", "{:+3.2f}".format(roll*57.32), "{:+3.2f}".format(pitch*57.32), "{:+3.2f}".format(yaw*57.32)
		#print roll,pitch
		#print accels[0]
		#print xpos,ypos,zpos
		#print "Analogs:", analog[0], analog[1], analog[2], analog[3], analog[4]
		#print "Altitude:", current_alt
		#print pitch_angle_gyro
		#print roll_angle_acc
		#print roll_angle_gyro
		#if GPS_data is not None:
		#	print "Location:", "{:+3.6f}".format(GPS_data.lat/10000000.0), "{:+3.6f}".format(GPS_data.lon/10000000.0), "{:+4.1f}".format(GPS_data.heightSea/1000.0)
		#	print "Loc Accuracy:", "{:+3.3f}".format(GPS_data.horAcc/1000.0), "{:+3.3f}".format(GPS_data.verAcc/1000.0)
		#print pitch_angle_gyro
		#print accels
		timer_1hz = current_time
		# End of 1Hz section
