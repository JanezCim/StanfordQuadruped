import numpy as np
import time
from src.Command import Command
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State, BehaviorState
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
import rospy
from geometry_msgs.msg import Twist 

ros_cmd_vel = Twist()

def cmd_vel_callback(msg):
    global ros_cmd_vel
    ros_cmd_vel = msg

def main(use_imu=False):
    """Main program
    """

    rospy.init_node('pupper', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    while not rospy.is_shutdown():
        # make sure there is at least dt time between previous and next loop
        now = time.time()
        if now - last_loop < config.dt:
            continue
        last_loop = time.time()
        command = Command()

        if config.read_joystick:
            print("Waiting for L1 to activate robot.")
            while True:
                command = joystick_interface.get_command(state)
                joystick_interface.set_color(config.ps4_deactivated_color)
                if command.activate_event == 1:
                    break
                time.sleep(0.1)
            print("Robot activated.")

            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
        # if reading joystick is disabled, read ROS commands instead
        else:
            # overwrite joystick commands with ROS commands
            command.horizontal_velocity = np.array([ros_cmd_vel.linear.x, ros_cmd_vel.linear.y])
            # command.horizontal_velocity[0] = float(0.5) #ros_cmd_vel.linear.x
            # command.horizontal_velocity[1] = ros_cmd_vel.linear.y
            command.yaw_rate = ros_cmd_vel.angular.z
            command.activation = True
            command.activate_event = False # since we dont have controller enabled, we always want activate event to be on false

            print(ros_cmd_vel)
        
            # trot only if moving command vels are all non-zero
            if ros_cmd_vel.linear.x == 0.0 and ros_cmd_vel.linear.y == 0.0 and ros_cmd_vel.angular.z == 0.0:
                state.behavior_state = BehaviorState.REST
            else:
                state.behavior_state = BehaviorState.TROT

            print("Behaviour:"+str(state.behavior_state))

        # Read imu data. Orientation will be None if no data was available
        quat_orientation = (
            imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        )
        state.quat_orientation = quat_orientation

        # Step the controller forward by dt
        controller.run(state, command)

        # Update the pwm widths going to the servos
        hardware_interface.set_actuator_postions(state.joint_angles)


main()
