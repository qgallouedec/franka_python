import rospy
from sensor_msgs.msg import Image
from franka_gym.utils import subscriber

from cv_bridge import CvBridge
cvbridge = CvBridge()

class RealSenseInterface:
    def __init__(self):
        """Constructor."""

        self.name = '/camera'

        ns = self.name + '/'

        self._image_raw = None
        self._joint_states_state_sub = subscriber(
            ns + 'color/image_raw', Image, self._image_raw_callback, tcp_nodelay=True)
    
    def _image_raw_callback(self, msg):
        cv_image = cvbridge.imgmsg_to_cv2(msg, "bgr8")
        self._image_raw = msg.data


if __name__=='__main__':
    rospy.init_node('test_node', anonymous=True, disable_signals=True)
    r = RealSenseInterface()
    import time
    time.sleep(2)
    print(r._image_raw)
