import sys
import time
import threading
import odrive
from odrive.enums import *

import rclpy
from sensor_msgs.msg import JointState

global_lock = threading.Lock()


class OdriveControl:
    def __init__(self, axis_nums, serial_number):
        self.odrive_serial_num = serial_number
        self.axis_nums = axis_nums
        self.odrv_axes = []

        print("Looking for ODrive...")
        self._find_odrive(serial_number)
        print("Found ODrive. serial number: {}".format(hex(self.odrv.serial_number).upper()))
        
    def _find_odrive(self, serial_num):
        # connect to Odrive
        self.odrv = odrive.find_any(serial_number=serial_num)

        for axis_num in range (0, self.axis_nums):
            self.odrv_axes.append(getattr(self.odrv, "axis{}".format(axis_num)))

    def configure(self, axis_num):
        # self.odrv_axes[axis_num].requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
        self.odrv_axes[axis_num].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("motor calibration ...", end=" ")
        while not(self.odrv_axes[axis_num].motor.is_calibrated):
            continue
        print("done!")
        print("encoder calibration ...", end=" ")
        while not(self.odrv_axes[axis_num].encoder.is_ready):
            continue
        print("done!")


        if self.odrv_axes[axis_num].motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.odrv_axes[axis_num].motor.error,
                                self.odrv_axes[axis_num].motor))
           
            sys.exit(1)

        if self.odrv_axes[axis_num].encoder.error != 0:
            print("Error: Odrive reported an error of {} while in the state "
            "AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            "data for debug:\n{}".format(self.odrv_axes[axis_num].encoder.error,
                                         self.odrv_axes[axis_num].encoder))
           
            sys.exit(1)

        print("Configuration for Axis{} is done".format(axis_num))
   
    def configure_motor(self, axis_num):
        self.odrv_axes[axis_num].requested_state = AXIS_STATE_MOTOR_CALIBRATION
        # self.odrv_axes[axis_num].requested_state = AxisState.MOTOR_CALIBRATION
    
    def configure_encoder(self, axis_num):
        self.odrv_axes[axis_num].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        # self.odrv_axes[axis_num].requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
    
    def mode_close_loop_control(self, axis_num):
        self.odrv_axes[axis_num].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # self.odrv_axes[axis_num].requested_state = AxisState.CLOSED_LOOP_CONTROL

    def mode_torque_control(self, axis_num):
        self.odrv_axes[axis_num].controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        # self.odrv_axes[axis_num].controller.config.control_mode = ControlMode.TORQUE_CONTROL
        self.odrv_axes[axis_num].controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        # self.odrv_axes[axis_num].controller.config.input_mode = InputMode.PASSTHROUGH

    def mode_position_control(self, axis_num):
        self.odrv_axes[axis_num].controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        # self.odrv_axes[axis_num].controller.config.control_mode = ControlMode.POSITION_CONTROL
        self.odrv_axes[axis_num].controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        # self.odrv_axes[axis_num].controller.config.input_mode = InputMode.PASSTHROUGH

    def mode_velocity_control(self, axis_num):
        self.odrv_axes[axis_num].controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        # self.odrv_axes[axis_num].controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv_axes[axis_num].controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        # self.odrv_axes[axis_num].controller.config.input_mode = InputMode.PASSTHROUGH

    def set_input_torq(self, axis_num, torque):
        self.odrv_axes[axis_num].controller.input_torque = torque

    def set_input_vel(self, axis_num, velocity):
        self.odrv_axes[axis_num].controller.input_vel = velocity

    def set_input_pos(self, axis_num, position):
        self.odrv_axes[axis_num].controller.input_pos = position

def joint_callback(msg):
    global header, name, input_pos, input_vel, input_torque
    header = msg.header
    name = msg.name
    input_pos = msg.position
    input_vel = msg.position
    input_torque = msg.effort

def joint_state_transfer():
    global pos, vel, torq, angle
    
    
    while(rclpy.ok()):
        pos = input_pos
        vel = input_vel
        torq = input_torque



def OdriveController():
    global bolt_odrv, control_mode, pos, vel, torq, angle

    while(rclpy.ok()):
        
        # Read encoder
        global_lock.acquire()
        for i in range(0,3):
            angle[i] = bolt_odrv[i].odrv_axes[0].encoder.pos_estimate
            angle[i+1] = bolt_odrv[i].odrv_axes[1].encoder.pos_estimate
        global_lock.release()
        
        # Write Input
        if control_mode == "position":
            bolt_odrv[0].set_input_pos(0,pos[0])
            bolt_odrv[0].set_input_pos(1,pos[1])
            bolt_odrv[1].set_input_pos(0,pos[2])
            bolt_odrv[1].set_input_pos(1,pos[3])
            bolt_odrv[2].set_input_pos(0,pos[4])
            bolt_odrv[2].set_input_pos(1,pos[5])
        elif control_mode == "velocity":
            bolt_odrv[0].set_input_vel(0,vel[0])
            bolt_odrv[0].set_input_vel(1,vel[1])
            bolt_odrv[1].set_input_vel(0,vel[2])
            bolt_odrv[1].set_input_vel(1,vel[3])
            bolt_odrv[2].set_input_vel(0,vel[4])
            bolt_odrv[2].set_input_vel(1,vel[5])
        elif control_mode == "torque":
            bolt_odrv[0].set_input_torq(0,torq[0])
            bolt_odrv[0].set_input_torq(1,torq[1])
            bolt_odrv[1].set_input_torq(0,torq[2])
            bolt_odrv[1].set_input_torq(1,torq[3])
            bolt_odrv[2].set_input_torq(0,torq[4])
            bolt_odrv[2].set_input_torq(1,torq[5])

def main(args=None):
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('odrive_ros_control')
    node.get_logger().info('odrive ros control node created.')

    control_mode = node.declare_parameter('control_mode', 'position').value

    assert isinstance(control_mode, str)

    node.get_logger().warn('control mode: ' + control_mode)

    # Subscriber
    joint_state_sub = node.create_subscription(JointState, "joint_state", joint_callback, 10)
    joint_state_pub = node.create_publisher(JointState, "encoder_value", 1)

    # 3 Odrive Controller ( 2 * 3 = 6DOF)
    bolt_odrv = []
    bolt_odrv.append(OdriveControl(axis_nums = 2, serial_number="208039755632"))
    # bolt_odrv.append(OdriveControl(axis_nums = 2, serial_number="208039755632"))
    # bolt_odrv.append(OdriveControl(axis_nums = 2, serial_number="208039755632"))
    
    for axis_num in range(0, 2):
        bolt_odrv[0].configure(axis_num)
        # bolt_odrv[1].configure(axis_num)
        # bolt_odrv[2].configure(axis_num)

    for axis_num in range(0, 2):
        if control_mode == "position":
            bolt_odrv[0].mode_position_control(axis_num)
            # bolt_odrv[1].mode_position_control(axis_num)
            # bolt_odrv[2].mode_position_control(axis_num)
        elif control_mode == "velocity":
            bolt_odrv[0].mode_velocity_control(axis_num)
            # bolt_odrv[1].mode_velocity_control(axis_num)
            # bolt_odrv[2].mode_velocity_control(axis_num)
        elif control_mode == "torque":
            bolt_odrv[0].mode_torque_control(axis_num)
            # bolt_odrv[1].mode_torque_control(axis_num)
            # bolt_odrv[2].mode_torque_control(axis_num)

        bolt_odrv.mode_close_loop_control(axis_num)


    
if __name__ == "__main__":
    main()