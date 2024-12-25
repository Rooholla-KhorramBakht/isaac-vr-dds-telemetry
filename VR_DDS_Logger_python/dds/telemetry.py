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
        
