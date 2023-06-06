import math
import threading
import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import crowd_navigation_core.utils
import crowd_navigation_msgs.msg
import numpy as np
from quadprog import solve_qp
import CommonVars
import time as c_time
if CommonVars.TRAJECTORY_SAVE:
    from scipy.io import savemat


class MotionGenerationManager:
    '''
    This class is responsible of the movement of Tiago based on regulation and using the CBF implemented by us
    '''
    def __init__(self):
        self.data_lock = threading.Lock()
        # self.tiago_velocity = crowd_navigation_core.utils.TiagoVelocity(0.0,0.0,0.0)

        # Non-RT data:
        self.robot_configuration_nonrt = crowd_navigation_core.utils.Configuration(
            0.0, 0.0, 0.0
        )

        self.all_tiago_positions = []
        self.all_predicted_positions = []
        self.actor_names = ['actor_{}'.format(i) for i in range(CommonVars.NUM_ACTORS)]
        self.all_actors_positions = {key: list() for key in self.actor_names}
        self.all_velocities = []
        self.prev_input = np.array([0.0,0.0])
        self.all_hs, self.all_dhs = [], []
        self.timezero = 0
        self.maximum_qp_time = 0.0

        self.target_switched = CommonVars.TARGET_POINT

        self.crowd_motion_prediction_stamped_nonrt = \
            crowd_navigation_core.utils.CrowdMotionPredictionStamped(
                rospy.Time.now(),
                'map',
                crowd_navigation_core.utils.CrowdMotionPrediction()
            )

        # RT data:
        self.robot_configuration = self.robot_configuration_nonrt
        self.crowd_motion_prediction_stamped = self.crowd_motion_prediction_stamped_nonrt

        # Subscribers:
        rospy.Subscriber(
            "/crowd_navigation/crowd_motion_prediction",
            crowd_navigation_msgs.msg.CrowdMotionPredictionStamped,
            self.crowd_motion_prediction_stamped_callback
        )

        rospy.Subscriber(
            "/gazebo/model_states",
            gazebo_msgs.msg.ModelStates,
            self.gazebo_model_states_callback
        )
    
    def crowd_motion_prediction_stamped_callback(self, crowd_motion_prediction_stamped_msg):
        crowd_motion_prediction_stamped_nonrt = \
            crowd_navigation_core.utils.CrowdMotionPredictionStamped.from_message(
                crowd_motion_prediction_stamped_msg
            )
        self.data_lock.acquire()
        self.crowd_motion_prediction_stamped_nonrt = \
            crowd_motion_prediction_stamped_nonrt
        self.data_lock.release()

    def saturate_input(self,input):
        if(input[0] > CommonVars.MAXV):
            input[0] = CommonVars.MAXV
        elif(input[0] < CommonVars.MAXV_NEG):
            input[0] = CommonVars.MAXV_NEG

        if(input[1] > CommonVars.MAXW):
            input[1] = CommonVars.MAXW
        elif(input[1] < CommonVars.MAXW_NEG):
            input[1] = CommonVars.MAXW_NEG
        return input

    def gazebo_model_states_callback(self, gazebo_model_states_msg):
        tiago_name = 'tiago'
        if tiago_name in gazebo_model_states_msg.name:
            husky_idx = gazebo_model_states_msg.name.index(tiago_name)
            p = gazebo_model_states_msg.pose[husky_idx].position
            q = gazebo_model_states_msg.pose[husky_idx].orientation
            robot_configuration_nonrt =  crowd_navigation_core.utils.Configuration(
                p.x,
                p.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
            )
            self.data_lock.acquire()
            self.robot_configuration_nonrt = robot_configuration_nonrt
            self.data_lock.release()

    def wrap_angle(self,theta):
        return math.atan2(math.sin(theta), math.cos(theta))


    def executeEightTraj(self,state,k1,k2,time):
        xc = 0
        yc = 0
        a = 5
        b = 2
        tiagox = state.x
        tiagoy = state.y
        tiagotheta = state.theta

        w = .1
        t= ((time-self.timezero) % (2*np.pi/w))

        x = a*np.cos(t*w) #a*np.sin(2*t*w) + xc
        dx = -a*w*np.sin(t*w)
        ddx = -a*(w**2)*np.cos(t*w)

        y = a*np.sin(w*t)
        dy = a*w*np.cos(t*w)
        ddy = -a*(w**2)*np.sin(t*w)
                
        if CommonVars.TRAJECTORY_SAVE:
            self.all_predicted_positions.append(np.array([x,y, time]))
            self.all_tiago_positions.append(np.array([self.robot_configuration.x, self.robot_configuration.y, time]))
        
        print("{} , {} , {}, {}, {}".format(time,self.timezero,t,x,y))


        #thetap=math.atan2(dy,dx)
        u1 = dx + k1*(x - (tiagox +(CommonVars.TIAGO_A*np.cos(tiagotheta))))
        u2 = dy + k2*(y - (tiagoy + (CommonVars.TIAGO_A*np.sin(tiagotheta))))
        vp = u1*np.cos(tiagotheta) + u2*np.sin(tiagotheta)
        wp = -u1*(np.sin(tiagotheta)/CommonVars.TIAGO_A) + u2*(np.cos(tiagotheta)/CommonVars.TIAGO_A)

        driving_velocity = vp  
        steering_velocity = wp

        return np.array((driving_velocity,steering_velocity))

    def regulation(self,state,target,k1,k2,k3,tol):
        '''
        Simple method to perform regularization provided during the lectures
        '''
        x = state.x
        y = state.y
        theta = state.theta
      
        # Desired configuration:
        try:
            x_d = target.position.x
            y_d = target.position.y
        except:
            x_d = target[0]
            y_d = target[1]
    
        theta_d = 0

        # Unicycle configuration in desired reference frame coordinates:
        x_r = math.cos(-theta_d) * (x - x_d) - math.sin(-theta_d) * (y - y_d)
        y_r = math.sin(-theta_d) * (x - x_d) + math.cos(-theta_d) * (y - y_d)
        theta_r = self.wrap_angle(-theta_d + theta)

        # Polar coordinates in relative coordinates:
        rho   = math.sqrt(math.pow(x_r, 2.0) + math.pow(y_r, 2.0))
        gamma = self.wrap_angle(math.atan2(y_r, x_r) - theta_r + math.pi)
        delta = self.wrap_angle(gamma + theta_r)

        # Feedback control:
        if abs(rho) < tol:
            driving_velocity = 0.0
            steering_velocity = 0.0
            swap = True

        else:
            driving_velocity  = k1 * rho * math.cos(gamma)
            steering_velocity = k2 * gamma + k1 * math.sin(gamma) * math.cos(gamma) / gamma * (gamma + k3 * delta)
            swap = False
        return np.array((driving_velocity,steering_velocity)),swap
        
    def log_values(self):
        output_dict = {}
        output_dict['real'] = np.array(self.all_tiago_positions)
        output_dict['humans'] = self.all_actors_positions
        output_dict['velocities'] = np.array(self.all_velocities)

        if CommonVars.TASK_TYPE == '8':
            output_dict['predicted'] = np.array(self.all_predicted_positions)
            output_dict['hs'] = np.array(self.all_hs)
            output_dict['dhs'] = np.array(self.all_dhs)
        if CommonVars.TASK_TYPE == 'REG':
            output_dict['hs'] = np.array(self.all_hs)
            output_dict['dhs'] = np.array(self.all_dhs)
        savemat('/home/antonio/Documents/catkin_ws/log_pos_mg.mat', output_dict)
        if CommonVars.MOTION_PRINT:
            print('Maximum QP time: ' + str(self.maximum_qp_time))

    def updateCBF(self,input,old_vel,state,obs_list,cbf_radius,cbf_alpha,a,time):
        start_time = c_time.clock()
        num_obstacles = len(obs_list)
        input_size = 2
        G = np.zeros((num_obstacles,input_size))
        b = np.zeros(num_obstacles)

        # Tiago state
        x = state.x
        y = state.y
        theta = state.theta

        v = old_vel[0]
        w = old_vel[1]
        dx = v*math.cos(theta)
        dy = v*math.sin(theta)

        h_save_vec = np.ones(CommonVars.K_CLUSTERS+2) * 999
        dh_save_vec = np.ones(CommonVars.K_CLUSTERS+2) * 999

        h_save_vec[-2] = time
        dh_save_vec[-2] = time

        


        for i in range(0,num_obstacles):
            xo = obs_list[i].position.x
            yo = obs_list[i].position.y
            dxo = obs_list[i].velocity.x
            dyo = obs_list[i].velocity.y            

            if(CommonVars.MOTION_PRINT):
                print("--------------------------------")
                print("obstacle: ",i)
                print("tiago: x",x, " y:",y, "theta:", theta, "dx: ", dx, "dy: ",dy, "v:", v, "w:", w)
                print("target: x",xo, " y:", yo, "dxo:", dxo, "dyo:", dyo)
           
            ha = (x - xo + a*math.cos(theta))**2 + (y - yo + a*math.sin(theta))**2 - cbf_radius**2
            dha = 2*(y - yo + a*math.sin(theta))*(v*math.sin(theta) - dyo + a*w*math.cos(theta)) - 2*(dxo - v*math.cos(theta) + a*w*math.sin(theta))*(x - xo + a*math.cos(theta))
            if CommonVars.TRAJECTORY_SAVE:
                h_save_vec[i] = ha
                dh_save_vec[i] = dha
            g_v = (2*math.cos(theta)*(x - xo + a*math.cos(theta)) + 2*math.sin(theta)*(y - yo + a*math.sin(theta)))#*v 
            g_w = (2*a*math.cos(theta)*(y - yo + a*math.sin(theta)) - 2*a*math.sin(theta)*(x - xo + a*math.cos(theta)))#*w
            extra =  (- 2*dxo*(x - xo + a*math.cos(theta)) - 2*dyo*(y - yo + a*math.sin(theta)))
            extra = cbf_alpha*ha + extra
            extra = -1.0*extra

            b[i] = np.array(extra,dtype='d')
            G[i] = np.array((np.array(g_v),np.array(g_w)),dtype='d')
            if(CommonVars.MOTION_PRINT):
                print("dh: ", dha)
                print("h: ", ha)
                print("-extra+alpha*h: ", extra)
                print(G[i])
                print(b[i])

        P = np.array(([1,0],[0,1e-1]),dtype='d') #minimize velocity more
        q = np.matmul(P,input)
        negative_vbound_left = np.array([1, 0])
        negative_vbound_right = np.array([CommonVars.MAXV_NEG])

        negative_wbound_left = np.array([0, 1])
        negative_wbound_right = np.array([CommonVars.MAXW_NEG])

        positive_vbound_left = np.array([-1, 0])
        positive_vbound_right = np.array([-CommonVars.MAXV])

        positive_wbound_left = np.array([0, -1])
        positive_wbound_right = np.array([-CommonVars.MAXW])

        G = np.vstack((G,negative_vbound_left, negative_wbound_left, positive_vbound_left, positive_wbound_left))
        b = np.append(b,[negative_vbound_right, negative_wbound_right,positive_vbound_right, positive_wbound_right])

        try:
            sol, f, xu, iters, lagr, iact = solve_qp(P, q, G.T, b)
            h_save_vec[-1] = 0
            dh_save_vec[-1] = 0
        except ValueError:
            print("WARNING! CONSTRAINT INCONSISTENT OR QP ERROR!")
            sol = np.array((0.0,0.0))
            h_save_vec[-1] = 1
            dh_save_vec[-1] = 1

        


        mod_input = sol

        if(CommonVars.MOTION_PRINT):
            print("SOL: " + str(sol))
            print("old input =", input)
            print("new input =", mod_input)
        
        if CommonVars.TRAJECTORY_SAVE:
            self.all_hs.append(h_save_vec)
            self.all_dhs.append(dh_save_vec)

        end_time = c_time.clock() - start_time
        if end_time > self.maximum_qp_time:
            self.maximum_qp_time = end_time
        return mod_input

    def start(self):
        rate = rospy.Rate(100) # 100 Hz
        if CommonVars.TRAJECTORY_SAVE:
            rospy.on_shutdown(self.log_values)

        # Setting up publishers:
        cmd_vel_publisher = rospy.Publisher(
            '/mobile_base_controller/cmd_vel',
            geometry_msgs.msg.Twist,
            queue_size=1
        )



        while not rospy.is_shutdown():
            time = rospy.get_time()

            # Copy non-RT data to RT data if possible:
            if self.data_lock.acquire(False):
                self.robot_configuration = self.robot_configuration_nonrt
                self.crowd_motion_prediction_stamped = self.crowd_motion_prediction_stamped_nonrt
                self.data_lock.release()

            obs_list = []
            # iterate thru the received obstacles from motion prediction module
            count_actors = 0
            for motion_prediction in self.crowd_motion_prediction_stamped.crowd_motion_prediction.motion_predictions:
                if CommonVars.TRAJECTORY_SAVE:
                    self.all_actors_positions['actor_{}'.format(count_actors)].append(np.array([motion_prediction.position.x, motion_prediction.position.y, time]))
                    count_actors += 1
                if(CommonVars.MOTION_PRINT):
                    print(' RECEIVED p: ({}, {}, {}), v: ({}, {}, {})'.format(
                        motion_prediction.position.x,
                        motion_prediction.position.y,
                        motion_prediction.position.z,
                        motion_prediction.velocity.x,
                        motion_prediction.velocity.y,
                        motion_prediction.velocity.z
                    ))
                if(np.array_equal(np.array([motion_prediction.position.x,
                    motion_prediction.position.y]), np.array([999,999]))):
                    pass
                else:
                    obs_list.append(motion_prediction)
            
            if(CommonVars.MOTION_PRINT):
                for i,motion_prediction in enumerate(obs_list):
                    print("IDX: ", i, "pos: ", motion_prediction.position, "vel: ", motion_prediction.velocity)
            
            # decide a ff-input vector
            ff_input = np.array([0,0])
            if CommonVars.TASK_TYPE == 'REG':
                ff_input,swap = self.regulation(self.robot_configuration,self.target_switched,CommonVars.REGULATION_K1,CommonVars.REGULATION_K2,CommonVars.REGULATION_K3,0.5)      

            if CommonVars.TASK_TYPE == 'FOLLOW':
                if(len(obs_list) > 0):
                   ff_input = self.regulation(self.robot_configuration,obs_list[0],1,1,1,0.01)

            if CommonVars.TASK_TYPE == '8':
                ff_input = self.executeEightTraj(self.robot_configuration,.12,.12,rospy.get_time())

            if CommonVars.TASK_TYPE == 'STILL':
                ff_input = np.array([0,0])

            if CommonVars.MOTION_PRINT:
                print("FF UNCLAMPED Velocity:", ff_input)
            old_ff_input = ff_input
            ff_input = self.saturate_input(ff_input)
            if not(np.array_equal(old_ff_input,ff_input)):
                print("SATURATION!")
                exit()


            #update velocities
            driving_velocity = ff_input[0]
            steering_velocity = ff_input[1]

            if CommonVars.MOTION_PRINT:
                print("FF Velocity:", ff_input)
            #if there are obstacles, then apply cbf
            if(len(obs_list) > 0):

                # save the starting time for the reference trajectory
                if(self.timezero == 0):
                    self.timezero = rospy.get_time()

                new_input = self.updateCBF(ff_input,self.prev_input,self.robot_configuration,obs_list,CommonVars.CBF_RADIUS,CommonVars.CBF_ALPHA,CommonVars.TIAGO_A,rospy.get_time())
                if CommonVars.MOTION_PRINT:
                    print("CBF Velocity:", new_input)
                new_input = self.saturate_input(new_input)
                if CommonVars.MOTION_PRINT:
                    print("CLAMPED Velocity:", new_input)
                driving_velocity = new_input[0]
                steering_velocity = new_input[1]
                self.prev_input = np.array((driving_velocity,steering_velocity))

            self.all_velocities.append(np.array([driving_velocity, steering_velocity, time]))
            # Create a twist ROS message:
            cmd_vel_msg = geometry_msgs.msg.Twist()
            cmd_vel_msg.linear.x = driving_velocity
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0
            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = steering_velocity

            # Publish a twist ROS message:
            cmd_vel_publisher.publish(cmd_vel_msg)

            rate.sleep()

def main():
    rospy.init_node('crowd_navigation_motion_generation', log_level=rospy.INFO)
    
    motion_generation_manager = MotionGenerationManager()
    motion_generation_manager.start()