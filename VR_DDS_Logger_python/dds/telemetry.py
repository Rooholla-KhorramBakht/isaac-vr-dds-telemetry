import time
import os
import numpy as np

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.pub import DataWriter
from cyclonedds.util import duration
from threading import Thread
from .PoseMsg import VRPose
from .FlexivState import FlexivStateMsg
# from .flexiv_messages import FlexivState
# from VR_DDS_Logger_python import ASSETS_PATH

def set_cyclonedds_config(interface_name):
    # Generate the XML configuration
    xml_content = f"""<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="{interface_name}" priority="default" multicast="default" />
      </Interfaces>
    </General>
  </Domain>
</CycloneDDS>"""

    # Create a temporary XML file
    cfg_file_path = os.path.join(os.path.dirname(__file__), 'cyclonedds.xml')
    with open(cfg_file_path, 'w') as f:
        f.write(xml_content)
    
    # Set the environment variable
    os.environ['CYCLONEDDS_URI'] = f'file://{cfg_file_path}'

class VRPosePublihser:
    def __init__(self, topic_name, interface_name = None):
        if interface_name is not None:
            set_cyclonedds_config(interface_name)
        self.topic_name = topic_name
        self.participant = DomainParticipant()
        self.topic = Topic(self.participant, self.topic_name, VRPose)
        self.writer = DataWriter(self.participant, self.topic)

    def send(self, state):
        self.writer.write(state)

class VRPoseSubscriber:
  def __init__(self, topic_name, interface_name = None):
    self.topic_name = topic_name
    if interface_name is not None:
        set_cyclonedds_config(interface_name)
    self.participant = DomainParticipant()
    self.topic = Topic(self.participant, self.topic_name, VRPose)
    self.reader = DataReader(self.participant, self.topic)
    self.state = None
    self.last_stamp = time.time()
    self.running = True
    self.receive_thread = Thread(target=self.receive)
    self.receive_thread.start()

  def receive(self):
    while self.running:
      for msg in self.reader.take_iter(timeout=duration(milliseconds=1.)):
          self.state = msg
          self.last_stamp = time.time()

  def qt2Pose(self, q, t):
    from scipy.spatial.transform import Rotation as R
    pose = np.eye(4)
    rot = R.from_quat(q).as_matrix()
    pose[:3, :3] = rot
    pose[:3, 3] = t 
    return pose

  def getState(self):
    if self.state is not None:
      world_T_head = self.qt2Pose(np.array(self.state.hmd_q), 
                                  np.array(self.state.hmd_t))
      world_T_left = self.qt2Pose(np.array(self.state.left_q),
                                  np.array(self.state.left_t))
      world_T_right = self.qt2Pose(np.array(self.state.right_q),
                                  np.array(self.state.right_t))
      return dict(stamp = self.last_stamp,
                  world_T_head=world_T_head,
                  world_T_left=world_T_left,
                  world_T_right=world_T_right)
    else:
      return None

  def close(self):
    self.running = False
    self.receive_thread.join()   


class FlexivStateSubscriber:
  def __init__(self, topic_name='flexiv_state', interface_name = None):
    self.topic_name = topic_name
    if interface_name is not None:
        set_cyclonedds_config(interface_name)
    self.participant = DomainParticipant()
    self.topic = Topic(self.participant, self.topic_name, FlexivStateMsg)
    self.reader = DataReader(self.participant, self.topic)

  def getState(self):
    state = None
    for msg in self.reader.take_iter(timeout=duration(milliseconds=1.)):
      state = msg
    if state is not None:
      return dict(q=msg.q, 
                  dq = msg.dq, 
                  tau = msg.tau
                  )
    else:
      return None

class FlexivStatePublisher:
    def __init__(self, topic_name='flexiv_state', interface_name = None):
        if interface_name is not None:
            set_cyclonedds_config(interface_name)
        self.topic_name = topic_name
        self.participant = DomainParticipant()
        self.topic = Topic(self.participant, self.topic_name, FlexivStateMsg)
        self.writer = DataWriter(self.participant, self.topic)

    def send(self, state):
        self.writer.write(state)