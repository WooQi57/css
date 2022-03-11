import enum
import os
import math
import time

from controller import Robot, Node, Field

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from rosgraph_msgs.msg import Clock


CAMERA_DIVIDER = 8  # every nth timestep an image is published, this is n
defaultpositions = [-0.05101498763939969, -0.4764168173419817, 0.006769736355572479, 0.8049079656668593, -0.45333041272529706, -0.04828855321310417, 
                    0.051024988764736795, 0.47641599257384537, -0.006771592248384993, -0.8049033115711886, 0.45332676273253447, 0.04830260921860783]
                    

class RobotController:
    def __init__(self, ros_active=True, robot='wolfgang', do_ros_init=True, external_controller=False, base_ns='',
                 recognize=False, camera_active=True):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param do_ros_init: Whether to call rospy.init_node (only used when ros_active is True)
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_active = ros_active
        self.recognize = recognize
        self.camera_active = camera_active
        if not external_controller:
            self.robot_node = Robot()
        self.walkready = [0] * 20
        self.time = 0
        self.clock_msg = Clock()
        self.motors = []
        self.sensors = []
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.switch_coordinate_system = True
        self.is_wolfgang = False
        self.pressure_sensors = None
        
        self.motor_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

        self.external_motor_names = self.motor_names
        self.sensor_suffix = "_sensor"
        # accel_name = "Accelerometer"
        # gyro_name = "Gyro"
        # camera_name = "Camera"

        # self.robot_node = self.supervisor.getFromDef(self.robot_node_name)
        for motor_name in self.motor_names:
            self.motors.append(self.robot_node.getDevice(motor_name))
            self.motors[-1].enableTorqueFeedback(self.timestep)
            self.sensors.append(self.robot_node.getDevice(motor_name + self.sensor_suffix))
            self.sensors[-1].enable(self.timestep)

        # self.accel = self.robot_node.getDevice(accel_name)
        # self.accel.enable(self.timestep)
        # self.gyro = self.robot_node.getDevice(gyro_name)
        # self.gyro.enable(self.timestep)

        # self.camera = self.robot_node.getDevice(camera_name)
        # self.camera_counter = 0
        # if self.camera_active:
        #     self.camera.enable(self.timestep*CAMERA_DIVIDER)
        # if self.recognize:
        #     self.camera.recognitionEnable(self.timestep)
        #     self.last_img_saved = 0.0
        #     self.img_save_dir = "/tmp/webots/images" +\
        #                         time.strftime("%Y-%m-%d-%H-%M-%S") +\
        #                         os.getenv('WEBOTS_ROBOT_NAME')
        #     if not os.path.exists(self.img_save_dir):
        #         os.makedirs(self.img_save_dir)

        # set zero points
        self.set_arms_zero()

        if self.ros_active:
            if base_ns == "":
                clock_topic = "/clock"
            else:
                clock_topic = base_ns + "clock"
            
            self.clock_publisher = rospy.Publisher(clock_topic, Clock, queue_size=1)
            self.pub_js = rospy.Publisher(base_ns + "joint_states", JointState, queue_size=1)            
            
            rospy.Subscriber("/joint_states", JointState, self.command_cb, queue_size=1)
            
    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_ros()

    def publish_ros(self):
        # self.publish_joint_states()
        self.publish_clock()
        
    def command_cb(self, command: JointState):        
        rospy.loginfo('wth')
        for i, name in enumerate(command.name):
            try:
                motor_index = self.motor_names.index(name)
                self.motors[motor_index].setPosition(command.position[i])
                if len(command.velocity) == 0 or command.velocity[i] == -1:
                    self.motors[motor_index].setVelocity(self.motors[motor_index].getMaxVelocity())
                else:
                    self.motors[motor_index].setVelocity(command.velocities[i])
                # if not len(command.accelerations) == 0:
                #     self.motors[motor_index].setAcceleration(command.accelerations[i])

            except ValueError:
                print(f"invalid motor specified ({name})")

    def set_arms_zero(self):
        positions = [0, 0, 0, 
                     0, 0]
        for i in range(0, 5):
            self.motors[i].setPosition(positions[i-2])

    def get_joint_state_msg(self, last_pos=[0 for i in range(8)]+defaultpositions):        
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        js.velocity = []
        for i in range(len(self.sensors)):
            js.name.append(self.external_motor_names[i])
            value = self.sensors[i].getValue()
            js.position.append(value)
            js.effort.append(self.motors[i].getTorqueFeedback())
            vel=(value - last_pos[i]) / (self.timestep / 1000)
            last_pos[i]=value
            js.velocity.append(vel)
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())
    
    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)
