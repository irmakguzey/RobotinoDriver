
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from time import sleep
from time import clock
import math
from threading import Thread

import RPi.GPIO as GPIO

leftCount = 0
rightCount = 0
midCount = 0

wantedLeft = 0
wantedRight = 0
wantedMid = 0

class WantedVelCalc(Thread):

	mainConst = 2.5

	xConst = 2.5
	yConst = 2.5
	zConst = 2.5

	xSmall = True
	ySmall = True
	zSmall = True

	wantedMsg = Twist()
	currentMsg = Twist()

	rightVel = 0.0
	leftVel = 0.0
	midVel = 0.0

	#wanted velocities of the motors in order to get there
	wantedLeft = 0.0
	wantedRight = 0.0
	wantedMid = 0.0

	cos30 = (3**(1/2.0)) / 2.0
	cos60 = 1.0/2.0

	machineRad = 0.05


	def __init__(self):

		
		#rospy.loginfo("in the init")
		Thread.__init__(self)
		rospy.Subscriber("wanted_vel", Twist, self.getWanted)


	def run(self):
		#r = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.changeConstants()
			#r.sleep()

	def getWanted(self, data):

		#print "in threads getwanted!!"

		if self.wantedMsg.linear.x != data.linear.x:
			self.xConst = self.mainConst
		if self.wantedMsg.linear.y != data.linear.y:
			self.yConst = self.mainConst
		if self.wantedMsg.angular.z != data.angular.z:
			self.zConst = self.mainConst

		self.wantedMsg.linear.x = data.linear.x
		self.wantedMsg.linear.y = data.linear.y
		self.wantedMsg.linear.z = data.linear.z

		self.wantedMsg.angular.x = data.angular.x
		self.wantedMsg.angular.y = data.angular.y
		self.wantedMsg.angular.z = data.angular.z

		#self.changeConstants()

	
	def changeConstants(self):
			
		oldX = self.xSmall
		oldY = self.ySmall
		oldZ = self.zSmall

		self.velCompare()

		if self.xSmall != oldX:
			self.xConst = (1/2.0)*self.xConst

		if self.ySmall != oldY:
			self.yConst = (1/2.0)*self.yConst

		if self.zSmall != oldZ:
			self.zConst = (1/2.0)*self.zConst

		

		self.currentMsg.linear.x = -self.leftVel*self.cos30 + self.rightVel*self.cos30
		self.currentMsg.linear.y = -self.leftVel*self.cos60 - self.rightVel*self.cos60 + self.midVel
		self.currentMsg.linear.z = 0

		self.currentMsg.angular.x = 0
		self.currentMsg.angular.y = 0
		self.currentMsg.angular.z = (self.leftVel + self.rightVel + self.midVel) / 10.0 #this is to be fixed


		global wantedLeft
		global wantedRight
		global wantedMid

		wantedLeft = self.leftVel
		wantedRight = self.rightVel
		wantedMid = self.midVel


	def velCompare(self):

		if self.currentMsg.linear.x < self.wantedMsg.linear.x:
			self.xSmall = True
			self.goForward()
		elif self.currentMsg.linear.x > self.wantedMsg.linear.x:
			self.xSmall = False
			self.goBack()

		if self.currentMsg.linear.y < self.wantedMsg.linear.y:
			self.ySmall = True
			self.goRight()
		elif self.currentMsg.linear.y > self.wantedMsg.linear.y:
			self.ySmall = False
			self.goLeft()

		if self.currentMsg.angular.z < self.wantedMsg.angular.z:
			self.zSmall = True
			self.turnLeft()
		elif self.currentMsg.angular.z > self.wantedMsg.angular.z:
			self.zSmall = False
			self.turnRight()


	def goForward(self):
		self.rightVel += self.xConst * self.cos30
		self.leftVel -= self.xConst * self.cos30

	def goBack(self):
		self.rightVel -= self.xConst * self.cos30
		self.leftVel += self.xConst * self.cos30

	
	def goRight(self):
		self.midVel += self.yConst
		self.rightVel -= self.yConst * self.cos60
		self.leftVel -= self.yConst * self.cos60

	
	def goLeft(self):
		self.midVel -= self.yConst
		self.rightVel += self.yConst * self.cos60
		self.leftVel += self.yConst * self.cos60

	
	def turnLeft(self):
		self.midVel += self.zConst
		self.rightVel += self.zConst
		self.leftVel += self.zConst

	
	def turnRight(self):
		self.midVel -= self.zConst
		self.rightVel -= self.zConst
		self.leftVel -= self.zConst

class FeedBack(Thread):

	def __init__ (self):

		Thread.__init__(self)

		self.motorSetup()

		self.lAVal = 0 #old value of the leftA pin
		self.lBVal = 0

		self.rAVal = 0
		self.rBVal = 0

		self.mAVal = 0
		self.mBVal = 0

		self.dummyLeft = 0 #to count every 90 degrees change, when this count is 4 or -4 than global countLeft should increment by one
		self.dummyRight = 0
		self.dummyMid = 0

		self.tryMidCount = 0

	def run(self):

		global rightCount
		global leftCount
		global midCount

		lastLeftA = GPIO.LOW
		lastLeftB = GPIO.LOW

		lastRightA = GPIO.LOW
		lastRightB = GPIO.LOW

		lastMidA = GPIO.LOW
		lastMidB = GPIO.LOW

		countL = GPIO.LOW
		countR = GPIO.LOW
		countM = GPIO.LOW

		while not rospy.is_shutdown():

			dummy = GPIO.input(self.leftA)
			if lastLeftA == GPIO.LOW and dummy == GPIO.HIGH:
				if GPIO.input(self.leftB) == GPIO.LOW:
					leftCount += 1
				else:
					leftCount -= 1
			lastLeftA = dummy

			dummy = GPIO.input(self.rightA)
			if lastRightA == GPIO.LOW and dummy == GPIO.HIGH:
				if GPIO.input(self.rightB) == GPIO.LOW:
					rightCount += 1
				else:
					rightCount -= 1
			lastRightA = dummy

			dummy = GPIO.input(self.midA)
			if lastMidA == GPIO.LOW and dummy == GPIO.HIGH:
				if GPIO.input(self.midB) == GPIO.LOW:
					midCount += 1
				else:
					midCount -= 1
			lastMidA = dummy


	def motorSetup(self):

		self.leftA = 29
		self.leftB = 31
		self.midA = 33
		self.midB = 32
		self.rightA = 38
		self.rightB = 40

		GPIO.setmode(GPIO.BOARD)

		GPIO.setup(self.leftA, GPIO.IN)
		GPIO.setup(self.leftB, GPIO.IN)

		GPIO.setup(self.rightA, GPIO.IN)
		GPIO.setup(self.rightB, GPIO.IN)

		GPIO.setup(self.midA, GPIO.IN)
		GPIO.setup(self.midB, GPIO.IN)


class PwmPublisher:

	#this class will take the left, right and mid velocities and increase motors until they are there


	LeftPin1 = 11
	LeftPin2 = 12
	LeftEnable = 13

	MidPin1 = 15
	MidPin2 = 16
	MidEnable = 18

	RightPin1 = 35
	RightPin2 = 36
	RightEnable = 37

	mainConst = 0.040

	lConst = mainConst
	rConst = mainConst
	mConst = mainConst

	lSmall = True
	rSmall = True
	mSmall = True

	
	leftPulse = 0.0
	rightPulse = 0.0
	midPulse = 0.0

	#these currents are to check whether the current velocities are smaller / bigger then global wanted velocities
	#when motors' velocities can be checked properly, these velocities will be received globally
	currLeft = 0.0
	currRight = 0.0
	currMid = 0.0

	#these constants are for keeping the last counts of the motors
	#they are used in calculating the velocities of the motors
	#they are assigned to countLeft/Right/Mid in every 0.1 seconds, 
	#so that in the next 0.1 seconds the difference bw traces and counts will give how much the motor turned in 0.1 seconds
	traceLeft = 0
	traceRight = 0
	traceMid = 0


	#variables below are for arranging when to decrease the velocity when there are no signals
	constBefDec = 10
	countDec = 0
	dummyDec = 0

	#it is getting calculated in the constructor
	#highest velocity depends on the max rpm of the motors and the wheels' radius
	highestVel = 0.0
	maxRpm = 1800 #for 12 volts

	wheelRad = 0.05 #5 cm
	machineRad = 0.5 #50 cm
	wheelFreq = 50

	#in order to calculate velocity every 1 second, meaning when this count is 10, velocity should be calculated
	velCount = 0

	#gearConstant is the ratio between a motor's round count and a wheel's rount count
	#in this driver it is 4*4=16
	gearConstant = 16.0


	def __init__(self):

		rospy.Subscriber("wanted_vel", Twist, self.getWanted)
		self.calculateMaxVel()
		self.motorSetup()

	def calculateMaxVel(self):
		self.highestVel = (self.maxRpm * 2 * math.pi * self.wheelRad) / 60.0
		
	
	def motorSetup(self):

		GPIO.setmode(GPIO.BOARD)

		GPIO.setup(self.LeftPin1, GPIO.OUT)
		GPIO.setup(self.LeftPin2, GPIO.OUT)
		GPIO.setup(self.LeftEnable, GPIO.OUT)
		self.pwmLeft = GPIO.PWM(self.LeftEnable, self.wheelFreq) #starts with 50 hz
		self.pwmLeft.start(0)

		GPIO.setup(self.RightPin1, GPIO.OUT)
		GPIO.setup(self.RightPin2, GPIO.OUT)
		GPIO.setup(self.RightEnable, GPIO.OUT)
		self.pwmRight = GPIO.PWM(self.RightEnable, self.wheelFreq) #starts with 50 hz
		self.pwmRight.start(0)

		GPIO.setup(self.MidPin1, GPIO.OUT)
		GPIO.setup(self.MidPin2, GPIO.OUT)
		GPIO.setup(self.MidEnable, GPIO.OUT)
		self.pwmMid = GPIO.PWM(self.MidEnable, self.wheelFreq) #starts with 50 hz
		self.pwmMid.start(0)

	def controlLeft(self):

		#TODO fastest the motor can get should be known
		#so that motorPwm should be calculated accordingly

		if self.leftPulse > 0:
			GPIO.output(self.LeftPin2, GPIO.HIGH)
			GPIO.output(self.LeftPin1, GPIO.LOW)

		else:
			GPIO.output(self.LeftPin2, GPIO.LOW)
			GPIO.output(self.LeftPin1, GPIO.HIGH)

		if (abs(self.leftPulse)) > self.highestVel :
			motorPwm = 100
		elif (abs(self.leftPulse) < 0.01):
			motorPwm = 0
		else :
			motorPwm = (abs(self.leftPulse))*100.0/self.highestVel  #motor will start running with this velocity
		
		#print self.leftPulse
		self.pwmLeft.ChangeDutyCycle(motorPwm)

	def controlRight(self):

		if self.rightPulse > 0:
			GPIO.output(self.RightPin2, GPIO.HIGH)
			GPIO.output(self.RightPin1, GPIO.LOW)

		else:
			GPIO.output(self.RightPin2, GPIO.LOW)
			GPIO.output(self.RightPin1, GPIO.HIGH)

		if (abs(self.rightPulse)) > self.highestVel :
			motorPwm = 100
		elif (abs(self.rightPulse) < 0.01):
			motorPwm = 0
		else :
			motorPwm = (abs(self.rightPulse))*100.0/self.highestVel  #motor will start running with this velocity
		
		#print self.rightPulse
		self.pwmRight.ChangeDutyCycle(motorPwm)

	def controlMid(self):

		if self.midPulse > 0:
			GPIO.output(self.MidPin2, GPIO.HIGH)
			GPIO.output(self.MidPin1, GPIO.LOW)

		else:
			GPIO.output(self.MidPin2, GPIO.LOW)
			GPIO.output(self.MidPin1, GPIO.HIGH)

		if (abs(self.midPulse)) > self.highestVel :
			motorPwm = 100
		elif (abs(self.midPulse) < 0.01):
			motorPwm = 0
		else :
			motorPwm = (abs(self.midPulse))*100.0/self.highestVel  #motor will start running with this velocity

		#print self.midPulse
		self.pwmMid.ChangeDutyCycle(motorPwm)

	

	def getWanted(self, data):

		#this method in this class is only to decrease the velocity after a while

		self.countDec = 0
	
	def changeConstants(self):

		self.calcVels()		
		
		self.countDec += 1 

		#since at every changeconstant 1 is added to countDec a not >=, only >
		if self.countDec > self.constBefDec: 	

			if abs(self.leftPulse) < 0.1:
				self.leftPulse /= 2
			else:
				if self.leftPulse < 0:
					self.leftPulse += self.mainConst
				elif self.leftPulse > 0:
					self.leftPulse -= self.mainConst


			if abs(self.rightPulse) < 0.1:
				self.rightPulse /= 2
			else:
				if self.rightPulse < 0:
					self.rightPulse += self.mainConst
				elif self.rightPulse > 0:
					self.rightPulse -= self.mainConst


			if abs(self.midPulse) < 0.1:
				self.midPulse /= 2
			else:
				if self.midPulse < 0:
					self.midPulse += self.mainConst
				elif self.midPulse > 0:
					self.midPulse -= self.mainConst


			self.lConst = self.mainConst
			self.rConst = self.mainConst
			self.mConst = self.mainConst

		else:
		
			oldLeft = self.lSmall
			oldRight = self.rSmall
			oldMid = self.mSmall

			self.velCompare()

			if self.lSmall != oldLeft:
				self.lConst = (1/2.0)*self.lConst

			if self.rSmall != oldRight:
				self.rConst = (1/2.0)*self.rConst

			if self.mSmall != oldMid:
				self.mConst = (1/2.0)*self.mConst
	
		self.controlLeft()
		self.controlRight()
		self.controlMid()



	#since changeConstants is being called with 10 Hz, 10 times in a second
	#diff bw traces and counts will give how much each motor turned in 0.1 seconds
	def calcVels(self):
				
		"""
		self.currLeft = self.leftPulse
		self.currRight = self.rightPulse
		self.currMid = self.midPulse
		"""
		
		#diff bw counts and traces * 10 will give how much the motor turned in a second
		#16 is the gear constant between the motor and the wheels
		self.currLeft = (leftCount - self.traceLeft) / self.gearConstant *  2 * self.wheelRad * math.pi
		self.currRight = (rightCount - self.traceRight)  / self.gearConstant * 2 * self.wheelRad * math.pi
		self.currMid = (midCount - self.traceMid) / self.gearConstant * 2 * self.wheelRad * math.pi

		self.traceLeft = leftCount
		self.traceRight = rightCount
		self.traceMid = midCount
		

	def velCompare(self):

		if self.currLeft < wantedLeft:
			self.lSmall = True
			if wantedLeft < 0:
				if self.leftPulse + self.lConst < 0:
					self.leftPulse = self.leftPulse + self.lConst
				else:
					self.leftPulse = 0 #does the threshold value lConst make any difference?
			else:
				self.leftPulse = self.leftPulse + self.lConst
		elif self.currLeft > wantedLeft:
			self.lSmall = False
			if wantedLeft > 0:
				if self.leftPulse - self.lConst > 0:
					self.leftPulse = self.leftPulse - self.lConst
				else:
					self.leftPulse = 0
			else:
				self.leftPulse = self.leftPulse - self.lConst


		if self.currRight < wantedRight:
			self.rSmall = True
			if wantedRight < 0:
				if self.rightPulse + self.rConst < 0:
					self.rightPulse = self.rightPulse + self.rConst
				else:
					self.rightPulse = 0
			else:
				self.rightPulse = self.rightPulse + self.rConst

		elif self.currRight > wantedRight:
			self.rSmall = False
			if wantedRight > 0:
				if self.rightPulse - self.rConst > 0:
					self.rightPulse = self.rightPulse - self.rConst
				else:
					self.rightPulse = 0
			else:
				self.rightPulse = self.rightPulse - self.rConst


		if self.currMid < wantedMid:
			self.mSmall = True
			if wantedMid < 0:
				if self.midPulse + self.mConst < 0:
					self.midPulse = self.midPulse + self.mConst
				else:
					self.midPulse = 0
			else:
				self.midPulse = self.midPulse + self.mConst

		elif self.currMid > wantedMid:
			self.mSmall = False
			if wantedMid > 0:
				if self.midPulse - self.mConst > 0:
					self.midPulse = self.midPulse - self.mConst
				else:
					self.midPulse = 0
			else:
				self.midPulse = self.midPulse - self.mConst


	

	def stop(self):

		GPIO.output(self.LeftEnable, GPIO.LOW)
		GPIO.output(self.RightEnable, GPIO.LOW)
		GPIO.output(self.MidEnable, GPIO.LOW)

		print "stopping"

		self.pwmLeft.stop()
		self.pwmRight.stop()
		self.pwmMid.stop()


def main():
	
	
	rospy.init_node('pwm_publisher', anonymous=True);
	
	pwmPub = PwmPublisher()

	feedBackThread = FeedBack()
	feedBackThread.start()

	wantedVelThread = WantedVelCalc()
	wantedVelThread.start()

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		pwmPub.changeConstants()
		r.sleep()


	wantedVelThread.join()	
	feedBackThread.join()
	pwmPub.stop()
	GPIO.cleanup()

	
if __name__ == '__main__':
	
	try:
		main()
	except rospy.ROSInterruptException:
		pass
