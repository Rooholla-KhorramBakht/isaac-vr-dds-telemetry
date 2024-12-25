from VR_DDS_Logger_python.dds.telemetry import VRPoseSubscriber
from VR_DDS_Logger_python.visualization import RerunVisualizer
from VR_DDS_Logger_python.dds.preprocessors import PoseBasedPreprocessor
import time
import rerun as rr
import numpy as np
from scipy.spatial.transform import Rotation as R
import signal   

def on_exit(sig, frame):
    subscriber.close()
    exit(0)

visulizer = RerunVisualizer(app_name="RerunVisualizer")
subscriber = VRPoseSubscriber("vr_poses")
preprocessor = PoseBasedPreprocessor(subscriber)

signal.signal(signal.SIGINT, on_exit)

preprocessor.reset()
start = time.time()
while time.time()-start < 10:
    time.sleep(0.01)
    state = preprocessor.getState()
    if state is not None:
        world_T_head = state['world_T_head']
        world_T_left = state['world_T_left']
        world_T_right = state['world_T_right']

        visulizer.logCoordinateFrame(np.eye(4), '/world')
        visulizer.logCoordinateFrame(world_T_head, '/world/head')
        visulizer.logCoordinateFrame(world_T_left, '/world/left')
        visulizer.logCoordinateFrame(world_T_right, '/world/right')
        rr.log("left_switch", rr.Scalar(int(state['left_switch'])))
        rr.log("right_switch", rr.Scalar(int(state['right_switch'])))

subscriber.close()
