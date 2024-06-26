import rospy
import tf.transformations as tr
from bravo_ft_sensor.srv import FTReading, Calibration, CalibrationResponse
from sensor_msgs.msg import JointState

import numpy as np
from sklearn.linear_model import LinearRegression
import yaml

class FTCalibration:
    def __init__(self):
        self.g_base_frame = np.array(rospy.get_param("/force_torque/gravity_base_frame", [0,0,-9.8]))
        self.required_cali_pts = np.array(rospy.get_param("/force_torque/required_pts_for_calibration", 25))
        # set up ft reading
        print("FT Calibration: Waiting for service")
        rospy.wait_for_service('force_torque_reading')
        try:
            self.get_ft_reading = rospy.ServiceProxy("force_torque_reading", FTReading)
        except rospy.ServiceException as e:
            print("Issues with ft_reading service")
        
        # set up joint_state sub
        self.joint_angles = []
        self.joint_state_sub = rospy.Subscriber("/joint_state", JointState, self.joint_cb)
        
        self.get_reading_srv = rospy.Service("ft_calibration", Calibration, self.calibrationSRV)


    def joint_cb(self, msg):
        self.joint_angles = msg.position

    def getRotation(self, trans):
        #print("cali", trans)
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
        rot = tr.quaternion_matrix(q)[:3,:3] # rotation matrix (no translation)
        return rot

    def getRawData(self): # number of data points
        jntAngles = [] # list of joint angles
        gts = [] # list of transformed gravity (pre rotation)
        fs = [] # list of force readings
        ts = [] # list of torque readings
        # while not enough points
        while len(jntAngles) < self.required_cali_pts:
            inny = input(f"Ready to take ({len(jntAngles)}/{self.required_cali_pts} data sample? (q) to quit, (b) remove last data point\t")
            if inny == "q":
                return False, None, None, None, None
            elif inny == "b":
                del jntAngles[-1]
                del gts[-1]
                del fs[-1]
                del ts[-1]
                print("Removed last data point!\t")
            else:
                # store angles
                jntAngles.append(self.joint_angles)
                #print("cali", "Getting res")
                res = self.get_ft_reading(False)
                #print("\tcali Got ft reading")
                #print("cali", res)
                # store filtered (mean/median) force torque reading
                f = res.reading.force
                fs.append(np.matrix([ f.x, f.y, f.z]))
                t = res.reading.torque
                ts.append(np.matrix([t.x, t.y, t.z]))
                print(f"\tGot raw reading:{fs[-1]}, and {ts[-1]}")
                # get gravity in the pre-rotated ft sensor frame
                Rw = self.getRotation(res.world_to_ft_frame)
                g_raw_ft_frame = Rw @ self.g_base_frame
                gts.append(np.matrix(g_raw_ft_frame))
                #print("\tcali Got data point")
        return True, gts, fs, ts, jntAngles
                
    
    def LS(self, A, B):
        # perform least squares regression assuming Ax = B
        reg = LinearRegression(fit_intercept=False).fit(A,B)
        return reg.coef_, reg.score(A,B)

    def getRotLS(self, gt):
        # transformed gravity vector
        gx = float(gt[0,0])
        gy = float(gt[0,1])
        A = np.array([[gx, -gy],[gy, gx]])
        return A

    def estGRot(self, gts, fs):
        # perform least squares to estimate ft_frame rotation
        A = np.zeros((len(gts) * 2, 2))
        B = np.zeros((2*len(gts), 1))
        for i, gt in enumerate(gts):
            A[i*2:i*2+2, :] = self.getRotLS(gt)
            B[i*2:i*2+2, :] = fs[i][0,:2].T
        
        vals, r = self.LS(A,B)
        ct = float(vals[:,0])
        st = float(vals[:,1])
        theta = np.arctan2(st, ct)
        
        return float(theta), r # rotation and ls r

    def getForceLS(self, g):
        # transformed+rotated gravity
        A = np.zeros((3,4))
        A[:,0] = g.T
        A[:, 1:] = np.identity(3)
        return A
    
    def estM(self, gs, fs):
        A = np.zeros( (3*len(gs), 4) )
        B = np.zeros( (3*len(gs), 1) )
        for i, g in enumerate(gs):
            A[3*i:3*i+3, :] = self.getForceLS(g)
            B[3*i:3*i+3, :] = fs[i][0,:].T

        vals, r = self.LS(A, B)
        m = float(vals[:,0])
        Fbias = vals[:,1:]
        return m, Fbias, r # mass and biases and ls r

    def getTorqueLS(self, g, m):
        # transf/rot gravity and mass
        gx = m * g[0]
        gy = m * g[1]
        gz = m * g[2]
        A = np.zeros((3,6))
        A[0,1] = gz
        A[1,0] = -gz
        A[0,2] = -gy
        A[2,0] = gy
        A[1,2] = gx
        A[2,1] = -gx
        A[:,3:] = np.identity(3)
        return A

    def estCOG(self, gs, m, ts):
        A = np.zeros( (3*len(gs), 6) )
        B = np.zeros( (3*len(gs), 1) )
        for i, g in enumerate(gs):
            A[3*i:3*i+3, :] = self.getTorqueLS(g, m)
            B[3*i:3*i+3, :] = ts[i][0,:].T
        vals, r = self.LS(A,B)
        cog = vals[:,:3]
        Tbias = vals[:,3:]
        return cog, Tbias, r # cog and torque bias and ls r
    
    def rotGs(self, gts, theta):
        ct = np.cos(theta)
        st = np.sin(theta)
        R = np.matrix([[ct, -st, 0],[st, ct, 0],[0,0,1]])
        gs = np.zeros((len(gts), 3))
        for i, gt in enumerate(gts):
            gs[i,:3] = (R * gt.T).T

        return gs

    def calibrationSRV(self, req):
        success, gts, fs, ts, jntAngles = self.getRawData()
        if not success:
            res = CalibrationResponse()
            res.success = False
            res.r_values = [-1.0, -1.0, -1.0]
            return res
        
        #print(gts[0])
        if req.calibrate_rotation:
            # estimate gravity rotation
            theta, rt = self.estGRot(gts, fs)
            # estimate mass and force bias
            gs = self.rotGs(gts, theta) # rotated gts 
        else:
            theta = 0
            rt = -1
            gs = gts

        # estimate mass and force bias
        m, fBias, rm = self.estM(gs, fs)

        # estimate cog and torque bias
        cog, tBias, rcog = self.estCOG(gs, m, ts)
        
        # store calibration
        print(type(theta), type(rt), type(rm), type(rcog))
        cali_data = {"m":m, "COG":cog.tolist(), "Fbias":fBias.tolist(), "Tbias":tBias.tolist(), 
                     "Theta":theta, "r_theta":float(rt), "r_m":float(rm), "r_cog":float(rcog)}
        with open(req.save_filepath + "/calibration_data.yaml", 'w') as file:
            yaml.dump(cali_data, file)

        # store raw data
        raw_data = {"gravity_base_ft_frame":gts, "Forces":fs, "Torques":ts, "Joint_Angles":jntAngles}
        with open(req.save_filepath + "/raw_data.yaml", 'w') as file:
            yaml.dump(raw_data, file)

        # respond
        res = CalibrationResponse()
        res.success = True
        res.r_values = [rt, rm, rcog]


        print(f"Theta:{theta} with r=", "{:.5f}".format(rt))
        print(f"Mass:{m} with r=", "{:.5f}".format(rm))
        print(f"fBias:{fBias} with r=", "{:.5f}".format(rm))
        print(f"COG:{cog} with r=", "{:.5f}".format(rcog))
        print(f"tBias:{tBias} with r=", "{:.5f}".format(rcog))


        return res

    def readRawData(self, fp):
        print(fp+"/raw_data.yaml")
        with open(fp + "/raw_data.yaml", 'r') as file:
            raw_data = yaml.load(file, Loader=yaml.Loader)

        for thing in raw_data:
            print(f"Another Thing:{thing}")
            for val in raw_data[thing]:
                print(f"\t{val}")


    def spin(self):
        print("Force Torque Calibration Node initialized")
        rospy.spin()


def test():
    np.set_printoptions(precision=3, suppress=True)
    # define constants for test case
    theta = 90.0 * 3.14 / 180
    ct = np.cos(theta)
    st = np.sin(theta)
    Rt = np.matrix([[ct, -st, 0],[st, ct, 0],[0,0,1]])
    m = 10 
    cog = np.array([1,2,3])
    fBias = np.array([4,5,6])
    tBias = np.array([7,8,9])
    g = np.matrix([[0],[0],[-9.8]])
    from scipy.spatial.transform import Rotation as R

    # create test dataset
    gts = []
    Fs = []
    Ts = []
    n = 50
    Rws = R.random(n, 27)
    for i in range(n):
        # get random orientation
        Rw = Rws[i].as_matrix()
        
        #print("Random Orientation:\n", Rw)
        # rotate 
        Ro = Rt @ Rw
        #print("Rotated by theta:\n", Rt, "\n", Ro)
        # calculate actual force/torque of gravity
        F = (m * Ro * g).T
        #print("Raw F:\n", F)
        T = np.cross(cog, F)
        #print("Raw T:\n", T)
        # add bias
        F += fBias
        T += tBias
        #print(F,T)
        gt = Rw * g
        #print("Gt:\n", gt)
        gts.append(gt.T)
        Fs.append(F)
        Ts.append(np.matrix(T))
    print(type(gts[0]), type(Fs[0]), type(Ts[0]))
    print (gts[0])
    print(Fs[0])
    print(Ts[0])
    # estimate theta 
    cali = FTCalibration()
    theta_est, r = cali.estGRot(gts, Fs)
    print(f"Theta est:", "{:.4f}".format(theta_est), f"actual:{theta} with r=", "{:.5f}".format(r))

    ct = np.cos(theta_est)
    st = np.sin(theta_est)
    R_est = np.matrix([[ct, -st, 0],[st, ct, 0],[0,0,1]])
    gtrs = []
    for g in gts:
        gtrs.append(R_est * g.T)

    m_est, Fbias_est, rm = cali.estM(gtrs, Fs)
    print(f"M est:", "{:.2f}".format(m_est), f"actual:{m} with r=", "{:.5f}".format(rm))
    print(f"Fbias est:{Fbias_est}, actual:{fBias} with r=", "{:.5f}".format(rm))

    cog_est, Tbias_est, rt = cali.estCOG(gtrs, m_est, Ts)
    print(f"COG est:{cog_est}, actual:{cog} with r=", "{:.5f}".format(rt))
    print(f"Tbias est:{Tbias_est}, actual:{tBias} with r=", "{:.5f}".format(rt))


if __name__=="__main__":
    rospy.init_node("ft_calibration_node")
    FTC = FTCalibration()
    #FTC.readRawData("/home/hunter/catkin_ws")
    #test()
    FTC.spin()


        

