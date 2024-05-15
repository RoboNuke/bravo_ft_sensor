import rospy
import tf2_ros
import tf.transformations as tr
from bravo_ft_sensor.msg import FT_Interrupt
from geometry_msgs.msg import Wrench, WrenchStamped, Twist
import numpy as np
import math 

class FTSafetyReporter():
    def __init__(self):
        self.ft_topic = rospy.get_param("ft_safety_interrupt/ft_topic", "filtered_force_torque")
        self.interrupt_topic = rospy.get_param("ft_safety_interrupt/interrupt_topic", "ft_interrupt")
        self.recovery_frame = rospy.get_param("ft_safety_interrupt/direction_output_frame", "bravo_base_link")
        self.check_mag = rospy.get_param("ft_safety_interrupt/check_mag", True)
        self.check_dirs = rospy.get_param("ft_safety_interrupt/check_dirs", True)
        self.dir_max = rospy.get_param("ft_safety_interrupt/direction_max_allowable", [1,1,1,0.1,0.1,0.1])
        self.mag_max = rospy.get_param("ft_safety_interrupt/magnitude_max_allowable", [3, 0.3])

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.ft_sub = rospy.Subscriber("filtered_force_torque", WrenchStamped, self.ftSensorCB)
        self.interrupt_pub = rospy.Publisher("ft_sensor_interrupt", FT_Interrupt, queue_size=1)

    def wrenchToNumpy(self, wren):
        npy = np.zeros((6,))
        npy[0] = wren.force.x
        npy[1] = wren.force.y
        npy[2] = wren.force.z
        npy[3] = wren.torque.x
        npy[4] = wren.torque.y
        npy[5] = wren.torque.z
        return npy
    
    def ftSensorCB(self, msg):
        wren = self.wrenchToNumpy(msg.wrench)
        safe = True
        if self.check_mag:
            fmax = math.sqrt(wren[0] ** 2 + wren[1] **2 + wren[2] ** 2)
            tmax = math.sqrt(wren[3] ** 2 + wren[4] ** 2 + wren[5] ** 2)
            if fmax >= self.mag_max[0] or tmax >= self.mag_max[1]:
                safe = False
        
        if safe and self.check_dirs:
            for i in range(6):
                if wren[i] >= self.dir_max[i]:
                    safe = False
                    break

        if not safe:
            out_msg = FT_Interrupt()
            out_msg.sensor_reading = msg.wrench

            # move wrench to recovery  frame
            tfWren = self.tfWrench(wren, msg.header.frame_id)
            
            # normalize wrench
            fTwist = tfWren[:3] / np.linalg.norm(wren[:3])
            tTwist = tfWren[3:] / np.linalg.norm(wren[3:])

            # take negative
            out_msg.recovery_direction.linear.x = -fTwist[0]
            out_msg.recovery_direction.linear.y = -fTwist[1]
            out_msg.recovery_direction.linear.z = -fTwist[2]
            out_msg.recovery_direction.angular.x = -tTwist[0]
            out_msg.recovery_direction.angular.y = -tTwist[1]
            out_msg.recovery_direction.angular.z = -tTwist[2]

            self.interrupt_pub.publish(out_msg)



    def tfWrench(self, wren, wren_frame):
        if wren_frame == self.recovery_frame:
            return wren
        
        trans = self.tfBuffer.lookup_transform(self.recovery_frame, wren_frame, rospy.Time())
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
        trans = tr.quaternion_matrix(q)
        lin = np.ones((4,1))
        rot = np.ones((4,1))
        lin[:3,0] = wren[:3]
        rot[:3,0] = wren[3:]
        newRot = trans @ rot
        trans[0:3, -1] = p
        #print("Trans:\n", trans)
        newLin = trans @ lin   

        out = np.zeros((6,))
        out[:3] = newLin[:3,0]
        out[3:] = newRot[:3,0]

        return out

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ft_safety_reporter")
    np.set_printoptions(precision=3, suppress=True)
    FTSR = FTSafetyReporter()
    FTSR.spin()


