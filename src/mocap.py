#!/usr/bin/env python
import rospy
import reef_msgs
from .quat import Quat
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist,TransformStamped
from std_msgs.msg import Float32MultiArray

class get_mocap:
    def __init__(self):
        # rospy.init_node("mocap_for_error")
        self._RPY_pub = rospy.Publisher("RPY",Float32MultiArray, queue_size=1)
        self._error_pub = rospy.Publisher("error",Float32MultiArray, queue_size=1)
        self._error_norm_pub = rospy.Publisher("error_norm",Float32MultiArray, queue_size=1)
        self._Vout_pub = rospy.Publisher("Vout",Float32MultiArray, queue_size=1)
        self._Vout_2d_pub = rospy.Publisher("Vout_2d",Float32MultiArray, queue_size=1)
        self._mocap_sub =rospy.Subscriber("/et/nwu/pose",TransformStamped,self.mocap_cb)
        # self.
        self.R=0
        self.Rd=0
        self.P=0
        self.Pd=0
        self.Y=0
        self.Yd=0
        self.tx=0
        self.ty=0
        self.tz=0
        self.ex=0
        self.ey=0
        self.ez=0
        self.en=np.array([0,0,0])
        self.en_2d=np.array([0,0])
        self.V=2 #m/s
        self.Vout=np.array([0,0,0])
        self.Vout_2d=np.array([0,0])
    def mocap_cb(self,mocap_state):
        # print(self.tx)
        self.R,self.P,self.Y=self.euler_from_quaternion(mocap_state.transform.rotation.x,mocap_state.transform.rotation.y,mocap_state.transform.rotation.z,mocap_state.transform.rotation.w)
        self.Rd=np.rad2deg(self.R)
        self.Pd=np.rad2deg(self.P)
        self.Yd=np.rad2deg(self.Y)
        self.ex=float(self.tx-mocap_state.transform.translation.x)
        self.ey=float(self.ty-mocap_state.transform.translation.y)
        self.ez=float(self.tz-mocap_state.transform.translation.z)
        self.en = np.array([self.ex,self.ey,self.ez]) / np.linalg.norm(np.array([self.ex,self.ey,self.ez]))
        self.en_2d = np.array([self.ex,self.ey]) / np.linalg.norm(np.array([self.ex,self.ey]))
        Cyaw=self.get_Cyaw(self.Y)
        Cyaw_2d=self.get_Cyaw_2d(self.Y)
        ebln=np.matmul(Cyaw,self.en)
        ebln_2d=np.matmul(Cyaw_2d,self.en_2d)
        self.Vout=np.array(np.multiply(self.V,ebln))[0]
        self.Vout_2d=np.array(np.multiply(self.V,ebln_2d))[0]
        identity_quat = Quat([0, 0, 0, 1])
        print(identity_quat)
        



    def spin(self):
        msg=Float32MultiArray()
        msg2=Float32MultiArray()
        msg3=Float32MultiArray()
        msg4=Float32MultiArray()
        msg5=Float32MultiArray()
        msg5.data=self.Vout_2d
        msg4.data=self.Vout
        msg3.data=self.en
        msg2.data=[self.ex,self.ey,self.ez]
        msg.data=[self.Rd,self.Pd,self.Yd]
        self._RPY_pub.publish(msg)
        self._error_pub.publish(msg2)
        self._error_norm_pub.publish(msg3)
        self._Vout_pub.publish(msg4)
        self._Vout_2d_pub.publish(msg5)

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_Cyaw(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , 1*np.sin(Y), 0]
                        ,[-1*np.sin(Y) , np.cos(Y) , 0]
                        ,[0 , 0 , 1]])
        return Cyaw

    def get_Cyaw_2d(self,Y):
        Cyaw=np.matrix([[np.cos(Y) , np.sin(Y)]
                        ,[-1*np.sin(Y) , np.cos(Y)]])
        return Cyaw


# if __name__ == "__main__":
#     get_mocap()