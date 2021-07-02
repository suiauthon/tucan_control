#!/usr/bin/env python3

import copy, time, sys
import serial

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped

class SerialIO:

  def __init__(self):
    # Serial io params
    self.port = rospy.get_param('port', '/dev/ttyUSB0')
    self.baudrate = rospy.get_param('baudrate', int(115200))
    # Define serial port
    self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate)
    self.serial_port.isOpen()

    # Ros loop params
    self.rate = rospy.get_param('~rate', 10000)

    # Initial message is 0
    self.io_msg = str.encode('+00+00')
    #self.io_msg = b'\x00'
    # Subscriber to io topic
    self.serial_io_sub = rospy.Subscriber('mm/serial_io', PointStamped, self.serialIoCallback, queue_size=1)
    self.pub = rospy.Publisher('mm/pose_ref', PointStamped, queue_size=1)
    self.new_ref = 0
    self.msg_pub = PointStamped()

  def run(self):

    print("Starting loop.")
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      self.msg_pub.header.stamp = rospy.get_rostime()
      self.pub.publish(self.msg_pub)
      #msg_old = self.io_msg
      #print("If check\n")
      if self.new_ref:
        #time.sleep(5)
        self.new_ref = 0
        print("Nova referenca\n")
        self.serial_port.write(self.io_msg)
        print("Poslano na serijalu\n")
        print(self.io_msg)

    # Close the serial port
    print("Closing serial port.")
    self.serial_port.close()

  def serialIoCallback(self, msg):
    self.msg_pub = msg
    
    if msg.point.x >= 0:
      predznak_x = '+' 
    else:
      predznak_x = '-'
      
    if msg.point.y >= 0:
      predznak_y = '+' 
    else:
      predznak_y = '-'

    if abs(msg.point.x) < 10:
      #broj = '00' + str(abs(int(msg.point.x)))
      broj_x = '0' + str(abs(int(msg.point.x)))
    else:
      broj_x = str(abs(int(msg.point.x)))
    if abs(msg.point.x) > 81:
      broj_x = '81'
    if abs(msg.point.y) < 10:
      #broj = '00' + str(abs(int(msg.point.x)))
      broj_y = '0' + str(abs(int(msg.point.y)))
    else:
      broj_y = str(abs(int(msg.point.y)))
    if abs(msg.point.y) > 81:
      broj_y = '81'
    command = 'A' + predznak_x + broj_x + predznak_y + broj_y + 'B'

    self.new_ref = 1
    self.io_msg = str.encode(command)
    #print("Poslano na serijalu\n")
    #self.io_msg = b'\x0001'#str.encode('0000')
    #print("Komanda: ")
    print(command)
    #print("\n")

if __name__ == '__main__':
  rospy.init_node('serial_io')
  serial_io = SerialIO()
  serial_io.run()