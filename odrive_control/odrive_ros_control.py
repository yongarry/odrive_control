import sys
import time
import odrive
from odrive.enums import *

import rclpy

class OdriveControl:
    def __init__(self, axis_nums, serial_number):
        self.axis_nums = axis_nums
        self.odrv_axes = []

        print("Looking for ODrive...")
        self._find_odrive(serial_number)
        print("Found ODrive. serial number: {}".format(self.odrv.serial_number))
        
    def _find_odrive(self, serial_number):
        # connect to Odrive
        self.odrv = odrive.find_any(serial_number=serial_number)
        
        for axis_num in range (0, self.axis_nums):
            self.odrv_axes.append(getattr(self.odrv, "axis{}".format(serial_number)))

    def configure(self, axis_num):
        # self.odrv_axes[axis_num].requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
        self.odrv_axes[axis_num].requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        print("motor calibration ...")
        while not(self.odrv_axes[axis_num].motor.is_calibrated):
            continue
        print("done!")
        print("encoder calibration ...")
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

    def set_input_torque(self, axis_num, torque):
        self.odrv_axes[axis_num].controller.input_torque = torque

    def set_input_vel(self, axis_num, velocity):
        self.odrv_axes[axis_num].controller.input_vel = velocity

    def set_input_pos(self, axis_num, position):
        self.odrv_axes[axis_num].controller.input_pos = position

if __name__ == "__main__":
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('odrive_ros_control')
    node.get_logger().info('odrive ros control node created.')

    control_mode = node.declare_parameter('control_mode', 'position').value

    assert isinstance(control_mode, str)

    node.get_logger().warn('control mode: ' + control_mode)

    # 3 Odrive Controller ( 2 * 3 = 6DOF)
    bolt_odrv = OdriveControl(axis_nums = 2, serial_number="208039755632")
    # bolt_odrv1 = OdriveControl(axis_nums = 2, serial_number="208039755634")
    # bolt_odrv2 = OdriveControl(axis_nums = 2, serial_number="208039755632")
    
    for axis_num in range(0, 2):
        bolt_odrv.configure(axis_num)
        # bolt_odrv.configure(axis_num)
        # bolt_odrv.configure(axis_num)

    for axis_num in range(0, 2):
        if control_mode == "position":
            bolt_odrv.mode_position_control(axis_num)
            # bolt_odrv1.mode_position_control(axis_num)
            # bolt_odrv2.mode_position_control(axis_num)
        elif control_mode == "velocity":
            bolt_odrv.mode_velocity_control(axis_num)
            # bolt_odrv.mode_velocity_control(axis_num)
            # bolt_odrv.mode_velocity_control(axis_num)
        elif control_mode == "torque":
            bolt_odrv.mode_torque_control(axis_num)
            # bolt_odrv.mode_torque_control(axis_num)
            # bolt_odrv.mode_torque_control(axis_num)

        bolt_odrv.mode_close_loop_control(axis_num)

    

