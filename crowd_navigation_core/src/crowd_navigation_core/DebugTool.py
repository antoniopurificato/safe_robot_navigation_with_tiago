import random
import numpy as np
import pygame
import math
from pygame.locals import *
import threading
import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import crowd_navigation_core.utils
import crowd_navigation_msgs.msg
import CommonVars
import os
import datetime



def dashed_circle(surface, color, center, radius, dash_width=2):
    angle = 10
    for i in np.arange(0, 360, angle):
        start = (center[0] + radius * np.cos(np.radians(i)), 
                 center[1] + radius * np.sin(np.radians(i)))
        end = (center[0] + radius * np.cos(np.radians(i + angle / 2)), 
               center[1] + radius * np.sin(np.radians(i + angle / 2)))
        pygame.draw.line(surface, color, start, end, dash_width)

def create_color(list_of_colors):
    ret = []
    r = int(random.random() * 256)
    g = int(random.random() * 256)
    b = int(random.random() * 256)
    step = 256 / list_of_colors
    for i in range(list_of_colors):
        r += step
        g += step
        b += step
        r = int(r) % 256
        g = int(g) % 256
        b = int(b) % 256
        if(CommonVars.PROJECTION_MODE == False):
            ret.append((r, g, b))
        else:
            ret.append((r*0.7, g*0.7, b*0.7))
    return ret


# Screen
screen_width = 900; screen_height = 900
screen = pygame.display.set_mode([screen_width, screen_height], DOUBLEBUF)
# need to scale the units from gazebo because they are not in pixels
pixel_scale = 50
pygame.font.init()
font = pygame.font.Font('freesansbold.ttf', 16)

# Obstacles
num_obsts = 0
obstacle_radius = 5

# ROS variables
# non RT data
# this contains the robot configuration
global_robot_configuration_nonrt = crowd_navigation_core.utils.Configuration(
    0.0, 0.0, 0.0
)
#--------- GAZEBO UNITS, TO BE CONVERTED IN PIXEL UNITS!
cbf_radius = CommonVars.CBF_RADIUS #to be updated everytime we change it in motiongeneration
tiago_a = CommonVars.TIAGO_A #to be update 
#-----------

# this will be a dictionary containing the actor configurations
global_actor_configurations_nonrt = {}
num_actors = CommonVars.NUM_ACTORS
actor_names = ['actor_{}'.format(i) for i in range(num_actors)]

actor_colors = create_color(num_actors)

K_obstacles = CommonVars.K_CLUSTERS * (CommonVars.N_PREDICTIONS+1)
if(CommonVars.FAKE_FSM):
    print("FAKE FSM ACTIVATED! K_CLUSTERS IS NOW NUM_ACTORS")
    K_obstacles = CommonVars.NUM_ACTORS
obs_colors = create_color(K_obstacles)
global_crowd_motion_prediction_list_msg = None

class obstacle:
    '''
    Simple class used to show the obstacles in the debug;
    '''

    def __init__(self, conf, radius, color,name,screen,sceneheight):
        if (conf == {}):
            self.x = 0
            self.y = 0
        else:
            self.x = conf.x
            self.y = conf.y
        self.r = radius
        self.screen = screen
        self.color = color
        self.sceneheight = sceneheight
        self.name = name
        self.disabled = False

       
    def show(self,scale):
        # pygame draws y=0 at top, we want to be at the center of the window
        if(self.disabled):
            return
        drawx = ((self.sceneheight/2) + self.x * scale)
        drawy = ((self.sceneheight/2) - self.y * scale)
        pygame.draw.circle(self.screen, self.color, (int(drawx),int(drawy)), self.r)
        text = font.render(self.name, True, self.color)
        #textRect = text.get_rect()
        #textRect.center = (drawx // 2 + 1, drawy // 2 + 1)
        self.screen.blit(text, (drawx+60,drawy-60))
    


    def showCBF(self,radius,scale, dashed = False):
        if(self.disabled):
            return
        # pygame draws y=0 at top, we want to be at the center of the window
        radius = radius * scale # the radius is in gazebo units too
        drawx = (self.sceneheight/2)+ self.x*scale
        drawy = (self.sceneheight/2) - self.y*scale#should be self.sceneheight - self.y
        if( CommonVars.PROJECTION_MODE == False):
            wth = 2
        else:
            wth = 6 
        if dashed:
            dashed_circle(self.screen, self.color, (int(drawx),int(drawy)), radius, wth)
        else:
            pygame.draw.circle(self.screen, self.color, (int(drawx),int(drawy)), radius, wth)

    
    def update(self,data,dt):
        self.x = data.x
        self.y = data.y
        if(CommonVars.DEBUG_PRINT):
            print("obs:",self.name," x:",self.x," y:",self.y)



class unicycle:
    '''
    Simple class used to show the unycicle in the debug;
    '''

    def __init__(self, init_x, init_y, init_theta, robot_l, robot_b, a,screen,sceneheight):
        self.x = init_x
        self.y = init_y
        self.dx = 0
        self.dy = 0
        self.theta = init_theta
        self.l = robot_l # Robot length is 2*l
        self.b = robot_b # Robot breadth is 2*b
        self.X = np.array([self.x, self.y])
        self.a = a
        self.screen = screen
        self.sceneheight = sceneheight
      
        
    def show(self,scale):
        # pygame draws y=0 at top, we want to be at the center of the window
        drawx = ((self.sceneheight / 2) + self.x*scale)
        drawy = ((self.sceneheight/2) - self.y*scale) 
        drawtheta = -self.theta #should be -theta always because of pygame
        self.tip = [drawx + self.l * math.cos(drawtheta), drawy + self.l * math.sin(drawtheta)]
        self.bottom = [drawx - self.l * math.cos(drawtheta), drawy - self.l * math.sin(drawtheta)]
        self.bottom_l = [self.bottom[0] - self.b * math.sin(drawtheta), self.bottom[1] + self.b * math.cos(drawtheta)]
        self.bottom_r = [self.bottom[0] + self.b * math.sin(drawtheta), self.bottom[1] - self.b * math.cos(drawtheta)]
        controlpoint_x = drawx + self.a*math.cos(drawtheta)
        controlpoint_y = drawy + self.a*math.sin(drawtheta)
        
        pygame.draw.polygon(self.screen, (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)

        if(CommonVars.PROJECTION_MODE == False):
            circlewt = 1
            circlecolor = (255,255,0)
        else: 
            circlewt = 2
            circlecolor=(255,128,0)

        
        pygame.draw.circle(self.screen, circlecolor, (int(controlpoint_x),int(controlpoint_y)), circlewt)

    def update(self,data,dt):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta
        if(CommonVars.DEBUG_PRINT): print("robotx:",self.x,"roboty:",self.y,"robottheta:",self.theta)

    def showCBF(self,radius,scale):
        # pygame draws y=0 at top, we want to be at the center of the window
        radius = radius * scale # the radius is in gazebo units too
        drawx = (self.sceneheight/2)+ self.x*scale
        drawy = (self.sceneheight/2) - self.y*scale#should be self.sceneheight - self.y
        drawtheta = -self.theta #should be -theta always because of pygame
        controlpoint_x = drawx + self.a*math.cos(drawtheta)
        controlpoint_y = drawy + self.a*math.sin(drawtheta)

        if( CommonVars.PROJECTION_MODE == False):
            circlecolor=(255,255,255)
            wth = 2
        else:
            wth = 6
            circlecolor=(0,0,0)
        pygame.draw.circle(self.screen, circlecolor, (int(controlpoint_x),int(controlpoint_y)), radius, wth)
    

def gazebo_model_states_callback(gazebo_model_states_msg):
    global global_robot_configuration_nonrt
    global data_lock
    global global_actor_configurations_nonrt
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

    actor_configurations = {}
    for actor_name in actor_names:
        if actor_name in gazebo_model_states_msg.name:
            actor_idx = gazebo_model_states_msg.name.index(actor_name)
            p = gazebo_model_states_msg.pose[actor_idx].position
            q = gazebo_model_states_msg.pose[actor_idx].orientation
            actor_configuration = crowd_navigation_core.utils.Configuration(
                p.x,
                p.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
            )
            actor_configurations[actor_name] = actor_configuration
    data_lock.acquire()
    global_robot_configuration_nonrt = robot_configuration_nonrt
    global_actor_configurations_nonrt = actor_configurations
    data_lock.release()


def crowd_navigation_msgs_callback(crowd_motion_prediction_stamped_msg):
    global data_lock
    global global_crowd_motion_prediction_list_msg
    data_lock.acquire()
    global_crowd_motion_prediction_list_msg =  crowd_navigation_core.utils.CrowdMotionPredictionStamped.from_message(crowd_motion_prediction_stamped_msg)
    global_crowd_motion_prediction_list_msg = crowd_navigation_core.utils.CrowdMotionPrediction.from_message(global_crowd_motion_prediction_list_msg.crowd_motion_prediction)
    data_lock.release()




def main():
    global data_lock
    global global_robot_configuration_nonrt
    global global_actor_configurations_nonrt
    
    global global_crowd_motion_prediction_list_msg

    # rospy initialization
    rospy.init_node('crowd_navigation_debug_tool', log_level=rospy.INFO)
    rate = rospy.Rate(100) # 100 Hz
    data_lock = threading.Lock()

        

    # Subscribers:
    rospy.Subscriber(
        "/gazebo/model_states",
        gazebo_msgs.msg.ModelStates,
        gazebo_model_states_callback
    )

    rospy.Subscriber(
       'crowd_motion_prediction',
        crowd_navigation_msgs.msg.CrowdMotionPredictionStamped,
        crowd_navigation_msgs_callback
    )


    # PyGame inits
    pygame.init()
    pygame.display.set_caption('tiago simulator')
    clock = pygame.time.Clock()
    ticks = pygame.time.get_ticks()
    font = pygame.font.SysFont('Arial', 25)

    # Robot
    robot_x0 = 0; robot_y0 = 0; robot_theta0 = 0; robot_l = tiago_a*pixel_scale; robot_b = 6  # Initial position
    a = robot_l# distance of point for cbf
    robot = unicycle(robot_x0, robot_y0, robot_theta0, robot_l, robot_b, a,screen,screen_height)

    while(1):
        if(global_actor_configurations_nonrt == {} ):
            print("NO INFOS YET")
            rate.sleep()
            continue
        else:
            break
            
    # Obstacles
    actor_idx = 0
    obstacles_list={}
    for actor_name in actor_names:
        print(actor_name)
        if(global_actor_configurations_nonrt != {}):
            obstacles_list[actor_name] = obstacle(global_actor_configurations_nonrt[actor_name], obstacle_radius, actor_colors[actor_idx],actor_name,screen,screen_height)
            actor_idx+=1
    print("generated:", actor_idx, " obstacles!")
    print(obstacles_list)

    # Predicted
    obs_idx = 0
    default_position = global_robot_configuration_nonrt
    pred_obstacles_list=[]
    for i in range(0,K_obstacles):
        pred_obstacles_list.append(obstacle(default_position, obstacle_radius, obs_colors[i],'FSM_'+str(i),screen,screen_height))
        pred_obstacles_list[i].disabled = True
        obs_idx+=1
    print("generated:", obs_idx, " pred obstacles!")
    print(pred_obstacles_list)


     # PyGame loop
    while(1):
        # To exit
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
            
        if rospy.is_shutdown():
            break





  
        # FPS. Print if required
        # rospy sleep
        rate.sleep()
        dt = clock.tick(20) / 1000.0     # To limit fps, controls speed of the animation


        if(event.type == KEYDOWN and event.key == K_SPACE):
            now = datetime.datetime.now()
            filename = "./{}-{}-{}_{}_{}_{}.png".format(now.year, now.month, now.day, now.hour, now.minute, now.second)
            # Check if the file already exists and increment the filename if necessary
            i = 1
            while os.path.exists(filename):
                filename = "./{}-{}-{}_{}_{}_{}_{}.png".format(now.year, now.month, now.day, now.hour, now.minute, now.second, i)
                i += 1

            # Save the screenshot with the generated filename
            pygame.image.save(screen, filename)
            print("saved", filename)
                


        if (CommonVars.PROJECTION_MODE == False):
            screen.fill((50, 55, 60))   # background
        else:
            screen.fill((255, 255, 255))   # background
        

        #update robot and obstacle positions
        if data_lock.acquire(False):
                    
            if(global_actor_configurations_nonrt == {} or global_crowd_motion_prediction_list_msg=={}):
                print("NO INFOS YET")
                rate.sleep()
                continue
                
            if CommonVars.DEBUG_PRINT:
                print("TIAGO: ", "x: ", global_robot_configuration_nonrt.x, "y: ", global_robot_configuration_nonrt.y)
            robot.update(global_robot_configuration_nonrt,dt)
            for actor_name in actor_names:
                if(global_actor_configurations_nonrt != {}):
                    if(CommonVars.SHOW_REAL_PPL == False):
                        obstacles_list[actor_name].disabled = True
                    obstacles_list[actor_name].update(global_actor_configurations_nonrt[actor_name],dt)
                    print("ACTOR: ", i, "x: ", global_actor_configurations_nonrt[actor_name].x, "y: ", global_actor_configurations_nonrt[actor_name].y)

            if(global_crowd_motion_prediction_list_msg != None):
                for i, motion_prediction in enumerate(global_crowd_motion_prediction_list_msg.motion_predictions):
                    
               
                    if(np.array_equal(np.array([motion_prediction.position.x,
                    motion_prediction.position.y,
                    ]), np.array([999,999]))):
                        pred_obstacles_list[i].disabled = True# make predicted obstacle disappear from view
                    else:
                        pred_obstacles_list[i].disabled = False
                        pred_obstacles_list[i].update(motion_prediction.position,dt)
                    
                    if(CommonVars.DEBUG_PRINT):
                        print("IDX: ", i, "pos: ", motion_prediction.position, "vel: ", motion_prediction.velocity, "disabled:",pred_obstacles_list[i].disabled)

            data_lock.release()
        

        #show robot and obstacles
        robot.show(pixel_scale)
        robot.showCBF(CommonVars.CBF_DS,pixel_scale)
        for actor_name in actor_names:
            obstacles_list[actor_name].show(pixel_scale)
            obstacles_list[actor_name].showCBF(CommonVars.CBF_RHO,pixel_scale)
        for i in range(K_obstacles):
            pred_obstacles_list[i].show(pixel_scale)
            pred_obstacles_list[i].showCBF(CommonVars.CBF_RHO,pixel_scale, dashed = True)

        
        # drawing goal point
        if(CommonVars.TASK_TYPE == 'REG'):
            drawx = (screen_height/2)+ CommonVars.TARGET_POINT[0]*pixel_scale
            drawy = (screen_height/2) - CommonVars.TARGET_POINT[1]*pixel_scale#should be self.sceneheight - self.y
            pygame.draw.circle(screen, (255,0,0), (int(drawx),int(drawy)), 10, 2)

        # Update PyGame display
        pygame.display.flip()