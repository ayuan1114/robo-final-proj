"""panda_controller controller."""

from controller import Robot
from your_code_here import getDesiredRobotCommand
import roboticstoolbox as rtb
from spatialmath import SE3

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Get all motors (joint actuators)
motors = []
motors.append(robot.getDevice("panda_joint1"))
motors.append(robot.getDevice("panda_joint2"))
motors.append(robot.getDevice("panda_joint3"))
motors.append(robot.getDevice("panda_joint4"))
motors.append(robot.getDevice("panda_joint5"))
motors.append(robot.getDevice("panda_joint6"))
motors.append(robot.getDevice("panda_joint7"))

left = robot.getDevice('panda_finger::left')
right = robot.getDevice('panda_finger::right')

FORCE = 30 # N
CLOSED = 1e-5 # m
OPEN = 0.04 # m
left.setPosition(CLOSED)
right.setPosition(CLOSED)

# Main loop:
tt = 0
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    print(f"Timestep {tt}")
    desired_command = getDesiredRobotCommand(tt)
    
    # 7 Robot Joints
    for j, motor in enumerate(motors):
        motor.setPosition(desired_command[j])
        
    # Gripper
    if desired_command[-1]:
        left.setPosition(CLOSED)
        right.setPosition(CLOSED)
    else:
        left.setPosition(OPEN)
        right.setPosition(OPEN)
    tt+=1
