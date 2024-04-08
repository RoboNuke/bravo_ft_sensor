import rospy
from geometry_msgs.msg import WrenchStamped, Wrench
from bravo_ft_sensor.srv import FTReading

import numpy as np

class FTFilter:
    def __init__(self):
        self.raw_sub = rospy.Subscriber("/raw_force_torque", WrenchStamped, self.rawCB)
        self.filt_pub = rospy.Publisher("/filtered_force_torque", WrenchStamped, queue_size=1)

        self.get_reading_srv = rospy.Service("/force_torque_reading", FTReading, self.getReadingSrv)

        self.frame_angle = rospy.get_param("/force_torque/frame_angle", 0.0)
        self.cog =  np.array(rospy.get_param("/force_torque/ee_cog", np.array([0,0,0.0])))
        self.m = rospy.get_param("/force_torque/ee_mass", 0.0)
        self.bias = np.array(rospy.get_param("/force_torque/bias", np.array([0,0,0,0,0,0.])))
        self.ft_frame = rospy.get_param("/force_torque/frame_name", "bravo_ft_sensor_link")
        self.base_frame = rospy.get_param("/force_torque/base_frame", "world")


    def rawCB(self, msg):
        pass

    def getReadingSrv(self, req):
        pass



    


if __name__=="__main__":
    FTF = FTFilter()
    FTF.spin()