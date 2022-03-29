#!/usr/bin/env python3 
import roslib
roslib.load_manifest('sbg_driver')
import rospy

import sbg_driver.msg
import sensor_msgs.msg
from termcolor import colored
from threading import Lock
import copy

debug = '[sbg_driver::Sbg2Ros] '

class Sbg2Ros():
  def __init__(self):
    self.sub = rospy.init_node('Sbg2Ros')
    self.__sbg_name = rospy.get_param('~sbg_name', 'sbg')
    self.__name     = rospy.get_param('~name', 'sbg2ros')
    print(colored(debug+'subscribing to:', 'green'))
    rospy.Subscriber(f'{self.__sbg_name}/imu_data',
                     sbg_driver.msg.SbgImuData,
                     self.CB_imu_data)
    print(f'  {self.__sbg_name}/imu_data')
    rospy.Subscriber(f'{self.__sbg_name}/ekf_quat',
                     sbg_driver.msg.SbgEkfQuat,
                     self.CB_ekf_quat)
    print(f'  {self.__sbg_name}/ekf_quat')
    print(colored(debug+'publishing to:', 'green'))
    self.__PUB_imu = rospy.Publisher(f'{self.__sbg_name}/imu', 
                                     sensor_msgs.msg.Imu, 
                                     queue_size=10)
    print(print(f'  {self.__sbg_name}/imu'))
    self.__PUB_imu_raw = rospy.Publisher(f'{self.__sbg_name}/imu_raw', 
                                         sensor_msgs.msg.Imu, 
                                         queue_size=10)
    print(print(f'  {self.__sbg_name}/imu_raw'))
    self.__imu_data = []
    self.__ekf_quat = []
    self.__lock = Lock()

  def convert(self, imu_data, ekf_quat):
      imu = sensor_msgs.msg.Imu()
      imu_raw = sensor_msgs.msg.Imu()
      t_imu_data = imu_data.header.stamp.to_sec()
      t_ekf_quat = ekf_quat.header.stamp.to_sec()
      if t_imu_data<t_ekf_quat:
          imu.header = copy.copy(imu_data.header)
          imu_raw.header = copy.copy(imu_data.header)
      else:
          imu.header = copy.copy(ekf_quat.header)
          imu_raw.header = copy.copy(imu_data.header)
      # imu
      imu.angular_velocity    = copy.copy(imu_data.delta_angle)
      imu.linear_acceleration = copy.copy(imu_data.delta_vel)
      imu.orientation         = copy.copy(ekf_quat.quaternion)
      imu.orientation_covariance[0] = ekf_quat.accuracy.x**2
      imu.orientation_covariance[4] = ekf_quat.accuracy.y**2
      imu.orientation_covariance[8] = ekf_quat.accuracy.z**2
      # imu_raw
      imu_raw.angular_velocity    = copy.copy(imu_data.gyro)
      imu_raw.linear_acceleration = copy.copy(imu_data.accel)
      imu_raw.orientation         = copy.copy(ekf_quat.quaternion)
      imu_raw.orientation_covariance = copy.copy(imu.orientation_covariance)
      return imu, imu_raw

  def publish(self):
      self.__lock.acquire()
      for i in range(len(self.__imu_data)):
          for j in range(len(self.__ekf_quat)):
              if self.__ekf_quat[j].header.seq==self.__imu_data[i].header.seq:
                  # convert and remove it from list (using pop)
                  imu, imu_raw = self.convert(self.__imu_data.pop(i), self.__ekf_quat.pop(j))
                  self.__PUB_imu.publish(imu)
                  self.__PUB_imu_raw.publish(imu_raw)
                  self.__lock.release()
                  return
      self.__lock.release()

  def CB_imu_data(self, msg):
      self.__imu_data.append(msg)
      self.publish()
    
  def CB_ekf_quat(self, msg):
      self.__ekf_quat.append(msg)
      self.publish()
      
if __name__ == '__main__':
  my_node = Sbg2Ros()
  rospy.spin()
  