import rospy
import tf2_ros
import tf.transformations as tr
from geometry_msgs.msg import WrenchStamped, Wrench
from bravo_ft_sensor.srv import FTReading, FTReadingResponse

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
        self.filter_type = rospy.get_param("/force_torque/filter_type", "mean")
        self.g_base_frame = np.array([0,0,0,1.0])
        self.g_base_frame[:3] = np.array(rospy.get_param("/force_torque/gravity_base_frame", [0,0,-9.8]))[:3]

        # filtering stuff
        self.sample_size = rospy.get_param("/force_torque/filter_sample_size", 10)
        self.sample_idx = 0
        self.raw = np.zeros((6,self.sample_size))

        # tf stuff
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def wrenchToNumpy(self, wren):
        return np.array([wren.force.x, wren.force.y, wren.force.z, wren.torque.x, wren.torque.y, wren.torque.z])
    
    def numpyToWrench(self, wren):
        w = Wrench()
        w.force.x = wren[0]
        w.force.y = wren[1]
        w.force.z = wren[2]
        w.torque.x = wren[3]
        w.torque.y = wren[4]
        w.torque.z = wren[5]
        return w

    def getWorldToFrameTF(self, frame_name, just_trans_msg = False):
        trans = self.tfBuffer.lookup_transform(self.base_frame, frame_name, rospy.Time())
        if just_trans_msg:
            return trans
        #print("Transform :", trans)
        p = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        q = np.array([trans.transform.rotation.x, trans.transform.rotation.y,
                  trans.transform.rotation.z, trans.transform.rotation.w])
        norm = np.linalg.norm(q)
        if np.abs(norm - 1.0) > 1e-3:
            raise ValueError(
                "Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
                    str(q), np.linalg.norm(q)))
        elif np.abs(norm - 1.0) > 1e-6:
            q = q / norm
        g = tr.quaternion_matrix(q)
        g[0:3, -1] = p
        return g

    def getFilterOutput(self):
        if self.filter_type == "mean":
            return np.average(self.raw, axis=0)
        elif(self.filter_type == "median"):
            return np.median(self.raw, axis=0)
        else:
            return np.zeros((6,1))
    
    def getGravityFT(self):
        # gravity comp goes here
        gravity_tf = self.getWorldToFrameTF(self.ft_frame)
        #print(f"Gravity tf:{gravity_tf}")
        g = gravity_tf @ self.g_base_frame
        #print(f"New Gravity:{g}")

        F = self.m * g[:3]
        T = np.cross(self.cog, F[:3])
        #print(f"Gravity ft:{F},{T}")
        return F,T

    def getStampedWrench(self, filtered_wrench):
        out = WrenchStamped()
        out.header.frame_id = self.ft_frame
        out.header.stamp = rospy.Time.now()
        out.wrench = self.numpyToWrench(filtered_wrench)
        return out
    
    def rawCB(self, msg):
        new = self.wrenchToNumpy(msg.wrench)

        self.raw[self.sample_idx, :] = new
        self.sample_idx = (self.sample_idx + 1) % self.sample_size
        #print(f"Updated raw:{self.raw}")
        filtered_wrench = self.getFilterOutput()

        F,T = self.getGravityFT()

        #print(f"Filtered Wrench {filtered_wrench}")
        filtered_wrench[:3] -= (F + self.bias[:3])
        filtered_wrench[3:] -= (T + self.bias[3:])
        #print(f"Gravity comped filtered wrench {filtered_wrench}")

        out = self.getStampedWrench(filtered_wrench)

        print("Final Output:\n", out)
        self.filt_pub.publish(out)

    def getReadingSrv(self, req):
        out = self.getFilterOutput()

        if req.full_filter:
            F,T = self.getGravityFT()
            out[:3] -= (F + self.bias[:3])
            out[3:] -= (T + self.bias[3:])
        
        res = FTReadingResponse()
        res.reading = self.numpyToWrench(out)
        x = self.getWorldToFrameTF(self.ft_frame, True)
        #print(x)
        res.world_to_ft_frame = self.getWorldToFrameTF(self.ft_frame, True)
        #print(res.world_to_ft_frame)
        res.success = True
        #print("Returning srv in filter")
        return res
        
    def spin(self):
        print("Force Torque Filter Node initialized")
        rospy.spin()

import time
if __name__=="__main__":
    rospy.init_node("ft_filter_node")
    FTF = FTFilter()
    
    #np.set_printoptions(precision=3, suppress=True)
    #time.sleep(3)
    """
    Test calibration data:
        force_torque:
        in_use: true
        frame_angle: 0.0
        housing_length: 0.042
        ee_cog: [4.0, 5.0, 6.0]
        ee_mass: 10.0
        bias: [1.0,2.0,3.0,4.0,5.0,6.0]
        frame_name: "bravo_ft_sensor_link"
        base_frame: "world"
        filter_type: "median"
        gravity_base_frame: [1,2,-3.0]
    """
    """
    FTF.raw = np.array([[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6],[1,2,3,4,5,6]])
    FTF.filter_type = "mean"
    print(f"Mean Filtered: {FTF.getFilterOutput()}")
    FTF.filter_type = "median"
    print(f"Median Filtered: {FTF.getFilterOutput()}")

    test_msg = WrenchStamped()
    test_msg.wrench.force.x = -100
    test_msg.wrench.force.y = -100
    test_msg.wrench.force.z = -100
    test_msg.wrench.torque.x = -100
    test_msg.wrench.torque.y = -100
    test_msg.wrench.torque.z = -100

    FTF.rawCB(test_msg)

    test_req = FTReading()
    test_req.full_filter = False
    print(f"Partial srv call:{FTF.getReadingSrv(test_req).reading}")
    test_req.full_filter = True
    print(f"Full srv call:{FTF.getReadingSrv(test_req).reading}")
    """
    FTF.spin()