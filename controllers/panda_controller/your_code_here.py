start_pos = [0.0, 0.0, 0.0, -1.70, -1.57, 1.4, 0.785]

drop = [start_pos + [True],
[1, 1, 0.0, -0.5, -1.57, 1.4, 0.785, True],
[1, 1, 0.0, -0.5, -1.57, 1.4, 0.785, False],
start_pos + [False]]

grab1 = [[-0.17, 0.6, 0.0, -2.15, -1.57, 2.35, 0.785, False],
[-0.17, 0.6, 0.0, -2.15, -1.57, 2.35, 0.785, True]]
grab2 = [[-0.28, 0.9, 0.0, -1.595, -1.57, 2.32, 0.785, False],
[-0.28, 0.9, 0.0, -1.595, -1.57, 2.32, 0.785, True]]

arm_pos = []

arm_pos.append(start_pos + [False])
arm_pos.extend(grab1)
arm_pos.extend(drop)
arm_pos.extend(grab2)
arm_pos.extend(drop)

def getDesiredRobotCommand(tt):
    # tt: current simulation timestep
    # a timestep is 0.016s (62.5 Hz)

    # write your code here...
    # you should return a eight-element list
    # the first seven entries are the joint angles
    # the last entry is a bool for whether the gripper
    # should be closed
    
    # your goal is to pick-and-place:
    # grab both of the blocks (one at a time)
    # and put them in the plastic crate
    
    # some examples of commanding different joint locations
    # (remove as you complete your code)
    
    #return grab2[0]
    
    return arm_pos[min(len(arm_pos) - 1, tt // 50)]