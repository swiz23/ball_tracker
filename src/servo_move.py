#!/usr/bin/env python
# move servo mechanism
import serial
import time
from numpy import interp
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

class servo:

	def __init__(self):
		
		self.center_x = 640/2
		self.center_y = 480/2
		self.pan_chan = 4
		self.tilt_chan = 5
		self.width_conv = float(((2000-992)/90)*17)/self.center_x
		self.height_conv = float(((2000-992)/90)*17)/self.center_y
		self.dead_zone = 50
		self.currentPan = 1500
		self.currentTilt = 2000
		self.ser = serial.Serial('/dev/ttyACM1', 9600)
		initial_pan = self.pulse_width_to_bytes(self.pan_chan,self.currentPan)
		initial_tilt = self.pulse_width_to_bytes(self.tilt_chan,self.currentTilt)
		self.ser.write(initial_pan)
		self.ser.write(initial_tilt)
		self.move_servo = rospy.Subscriber('ball_center', Point, self.callback)

	def pulse_width_to_bytes(self,channel, width):
		#width = int(interp(angle,[0,90],[992,2000]))
		target = width*4
		data = [0x84, channel, target&0x7F, (target>>7)&0x7F]
		output = bytearray(data)
		return output

	def callback(self,data):
		
		rem_x = (data.x - self.center_x)
		rem_y = (data.y - self.center_y)
		
		#if rem_x > self.dead_zone or rem_x < -self.dead_zone:
		if rem_x > self.dead_zone and self.currentPan < 2000:
			self.currentPan = self.currentPan + 10
		elif rem_x < -self.dead_zone and self.currentPan > 992:
			self.currentPan = self.currentPan - 10
		outputCurrentPan = self.pulse_width_to_bytes(self.pan_chan,self.currentPan)
		self.ser.write(outputCurrentPan)
		if rem_y > self.dead_zone and self.currentTilt < 2000:
			self.currentTilt = self.currentTilt + 10
		elif rem_y < -self.dead_zone and self.currentTilt > 992:
			self.currentTilt = self.currentTilt - 10
		print(self.currentTilt)
		outputCurrentTilt = self.pulse_width_to_bytes(self.tilt_chan,self.currentTilt)
		self.ser.write(outputCurrentTilt)
		return

def main():
	rospy.init_node('servo_controller', anonymous=True)
	serv = servo()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		endPanConfig = serv.pulse_width_to_bytes(serv.pan_chan,1500)
		endTiltConfig = serv.pulse_width_to_bytes(serv.tilt_chan,2000)
		serv.ser.write(endPanConfig)
		serv.ser.write(endTiltConfig)
		serv.ser.close()
	return

if __name__ == '__main__':
  main()
