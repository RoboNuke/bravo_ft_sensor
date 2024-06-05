import rospy
from geometry_msgs.msg import WrenchStamped, TwistStamped, Twist
from std_srvs.srv import SetBool
from math import fabs


class FTDemoController:
    def __init__(self):
        self.ft_topic = rospy.get_param("ft_demo/force_torque_topic", 
                                        "/bravo/filtered_force_torque")
        self.control_topic = rospy.get_param("ft_demo/ee_control_topic", 
                                        "/bravo/ee_control")
        self.ee_frame = rospy.get_param("ft_demo/ee_frame", 
                                        "world")
        self.Ks = rospy.get_param("ft_demo/gains", 
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.threshs = rospy.get_param("ft_demo/ignore_threshold",
                                       [10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
        self.maxs = rospy.get_param("ft_demo/max_speeds",
                                       [0.5, 0.5, 0.5, 0.25, 0.25, 0.25])
        
        # set up service to toggle
        self.toggle_srv = rospy.Service("toggle_ft_demo_controller", 
                                        SetBool, self.toggleSrv)

        self.active = False
        self.ft_sub = rospy.Subscriber(self.ft_topic, 
                                       WrenchStamped, self.ftCB)
        self.twist_pub = rospy.Publisher(self.control_topic, 
                                         TwistStamped, queue_size=1)


    def toggleSrv(self, msg):
        if msg.data:
            # turn on
            self.active = True
            return True, "Force Torque Demo Control Activated"
        else:
            # turn off
            self.active = False

            out = TwistStamped()
            out.twist = self.toTwist([0,0,0,0,0,0])
            out.header.frame_id = self.ee_frame
            out.header.stamp = rospy.Time.now()
            self.twist_pub.publish(out)
            return True, "Force Torque Demo Control Deactivated"

    def toTwist(self, x):
        out = Twist()
        out.linear.x = x[0]
        out.linear.y = x[1]
        out.linear.z = x[2]
        out.angular.x = x[3]
        out.angular.y = x[4]
        out.angular.z = x[5]
        return out

    def ftCB(self, msg):
        if not self.active:
            return
        f = msg.wrench.force
        t = msg.wrench.torque
        ft = [f.x, f.y, f.z,
              t.x, t.y, t.z]
        
        vel = []
        for i in range(6):
            e = ft[i] if fabs(ft[i]) > self.threshs[i] else 0.0
            if e > 0.000001:
                e -= self.threshs[i]
            elif e < -0.000001:
                e += self.threshs[i]
            e*=self.Ks[i]
            e = min(self.maxs[i], max(-self.maxs[i], e))
            vel.append(e)

            

        out = TwistStamped()

        out.twist = self.toTwist(vel)
        out.header.frame_id = self.ee_frame
        out.header.stamp = rospy.Time.now()
        print("e:", e)
        self.twist_pub.publish(out)
        print("Pubbed:", out)
        
    def spin(self):
        print("Force-Torque Demo Controller Node initialized")
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("ft_demo_controller")
    ftDC = FTDemoController()
    ftDC.spin()




