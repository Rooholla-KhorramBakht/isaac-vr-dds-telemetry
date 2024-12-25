
import numpy as np
import time 

class PoseBasedPreprocessor:
    def __init__(self, subscriber, world_T_vrworld  = np.eye(4)):
        self.subscriber = subscriber    
        self.world0_T_world = np.eye(4)
        self.right_offset = np.eye(4)
        self.right_offset[:3,:3] = np.array([[ 0.97187117, -0.03434083, -0.23299599],
                                        [ 0.04319033,  0.9985222 ,  0.03298494],
                                        [ 0.23151894, -0.04212028,  0.97191813]])
        self.right_offset = np.linalg.inv(self.right_offset)
        self.left_offset = np.eye(4)
        self.left_offset[:3,:3] = np.array([[ 0.93723603,  0.27302218,  0.21690437],
                                    [-0.26480738,  0.96199454, -0.06665994],
                                    [-0.22686046,  0.00503822,  0.97391424]])
        self.left_offset = np.linalg.inv(self.left_offset)
        self.left_switch = False    
        self.right_switch = False
        self.world_T_vrworld = world_T_vrworld

    def reset(self):
        state = self.subscriber.getState()
        start = time.time()
        while state is None and time.time() - start < 5:
            state = self.subscriber.getState()
        if state is None:
            raise Exception("Failed to get initial state from the VR bridge. Make sure the DDS communication is working.")
        self.world0_T_world = np.linalg.inv(state['world_T_head']).copy()

    def getState(self):
        state = self.subscriber.getState()
        if state is not None:
            world_T_head = self.world0_T_world@state['world_T_head']
            world_T_left = self.left_offset@world0_T_world@state['world_T_left']
            world_T_right = self.right_offset@world0_T_world@state['world_T_right']

            global_y = np.array([0,1,0])
            left_y = world_T_left[:3,1]
            left_sw = np.dot(global_y, left_y)
            right_y = world_T_right[:3,1]
            right_sw = np.dot(global_y, right_y)
            if left_sw < 0.4:
                self.left_switch  = True
            elif left_sw > 0.8:
                self.left_switch  = False
            else:
                self.left_switch  = False

            if right_sw < 0.4:
                self.right_switch = True
            elif right_sw > 0.8:
                self.right_switch = False
            else:
                self.right_switch = False

            return dict(
                        world_T_head=self.world_T_vrworld@world_T_head, 
                        world_T_left=self.world_T_vrworld@world_T_left, 
                        world_T_right=self.world_T_vrworld@world_T_right,
                        left_switch=self.world_T_vrworld@self.left_switch, 
                        right_switch=self.world_T_vrworld@self.right_switch
                        )
        else: 
            return None