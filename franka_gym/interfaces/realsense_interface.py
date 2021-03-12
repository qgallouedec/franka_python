# -*- coding: utf-8 -*-

import threading
import copy
from typing import Tuple, List
from collections import deque

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from realsense2_camera.msg import Extrinsics
from bond.msg import Status

from franka_gym.utils import subscriber


class RealSenseInterface:
    """Interface with RealSense camera.

    Args:
        display (bool, optional): If True, a window opens with the live
            rendering of the cameras. Defaults to False.
        display_fps (float, optional): The number of frames per second of
            rendering. Defaults to 25.

    Example:
        >>> import rospy
        >>> rospy.init_node('realsense_node')
        >>> realsense = RealSenseInterface()
        >>> realsense.get_color_img().shape
        (480, 640, 3)
    """

    def __init__(
            self, buffer_size: int = 30, display: bool = False,
            display_fps: float = 25, display_depth_min: float = 0,
            display_depth_max: float = 2**16):
        """[summary]

        Args:
            buffer_size (int, optional): The number of previous frames that are
                returned by the method `get_color_imgs()`. Defaults to 30.
            display (bool, optional): If `True`, a window opens with the live
                rendering of the cameras. Defaults to False.
            display_fps (float, optional): The number of frames per second of
                rendering. Defaults to 25.
            display_depth_min (float, optional): The depth will appear as a
                colormap whose minimum value is determined by this argument.
                Defaults to 0.
            display_depth_max (float, optional): The depth will appear as a
                colormap whose maximum value is determined by this argument.
                Defaults to 2**16.
        """
        if rospy.get_name() == '/unnamed':
            raise Exception(
                'You must init a node before the interface. Call `rospy.init_node()`')
        self.ns = '/camera/'

        self.buffer_size = buffer_size
        self._color_img_buffer = deque([], self.buffer_size)

        self.display = display
        self.display_fps = display_fps
        self._build_display_depth_range(display_depth_min, display_depth_max)

        self._init_subscribers()

    def _build_display_depth_range(
            self, display_depth_min: int, display_depth_max: int):
        """Clip and store the display depth range.

        Args:
            display_depth_min (int): Minimal value of the displayed heatmap.
            display_depth_max (int): Maximal value of the displayed heatmap.

        Raises:
            ValueError: If the args do not comply to 0 < depth_min < depth_max < 2**16.
        """
        if display_depth_min >= display_depth_max:
            raise ValueError(
                '`display_depth_min` must be lower than `display_depth_max`')
        if display_depth_min < 0:
            raise ValueError('`display_depth_min` must be greater than 0')
        if display_depth_max > 2**16:
            raise ValueError('`display_depth_max` must be lower than 2**16')
        self.display_depth_min = display_depth_min
        self.display_depth_max = display_depth_max

    def _init_subscribers(self, timeout: float = 1) -> None:
        """Init subscribers.

        Args:
            timeout (float, optional): Wait for a message on topic during
                timeout seconds. Defaults to 1.
        """
        self._color_img = None
        self._color_img_sub = subscriber(
            self.ns + 'color/image_raw', Image, self._color_img_callback,
            timeout)

        self._depth_img = None
        self._depth_img_sub = subscriber(
            self.ns + 'depth/image_rect_raw', Image, self._depth_img_callback,
            timeout)

        self._color_camera_info = None
        self._color_camera_info_sub = subscriber(
            self.ns + 'color/camera_info', CameraInfo,
            self._color_camera_info_callback, timeout)

        self._depth_camera_info = None
        self._depth_camera_info_sub = subscriber(
            self.ns + 'depth/camera_info', CameraInfo,
            self._depth_camera_info_callback, timeout)

        self._extrinsics_depth_to_color = None
        self._extrinsics_depth_to_color_sub = subscriber(
            self.ns + 'extrinsics/depth_to_color', Extrinsics,
            self._extrinsics_depth_to_color_callback, timeout)

        self._realsense2_camera_manager_bond = {}
        self._realsense2_camera_manager_bond_sub = subscriber(
            self.ns + 'realsense2_camera_manager/bond', Status,
            self._realsense2_camera_manager_bond_callback, timeout)

        if self.display:
            threading.Thread(target=self._show, daemon=True).start()

    # Callbacks
    # region

    def _color_camera_info_callback(self, msg: CameraInfo) -> None:
        """Store the last color camera info in a private attribut.

        Info:
            More info http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html

        Args:
            msg (CameraInfo): The message from the topic.
        """
        self._color_camera_info = {
            'height': msg.height,
            'width': msg.width,
            'distortion_model': msg.distortion_model,
            'distortion_parameters': msg.D,
            'intrinsic_camera_matrix': msg.K,
            'rectification_matrix': msg.R,
            'extrinsic_camera_matrix': msg.P,
            'binning_x': msg.binning_x,
            'binning_y': msg.binning_y,
            'roi_x_offset': msg.roi.x_offset,
            'roi_y_offset': msg.roi.y_offset,
            'roi_height': msg.roi.height,
            'roi_width': msg.roi.width,
            'roi_do_rectify': msg.roi.do_rectify}

    def _color_img_callback(self, msg: Image) -> None:
        """Store the last color image in a private attribut.

        Args:
            msg (Image): The message from the topic.
        """
        flatten_color_img_bgr = np.frombuffer(msg.data, dtype=np.uint8)
        color_img_bgr = flatten_color_img_bgr.reshape(
            msg.height, msg.width, -1)
        color_img_rgb = color_img_bgr[..., ::-1]
        self._color_img = color_img_rgb
        self._color_img_buffer.append(color_img_rgb)

    def _depth_camera_info_callback(self, msg: CameraInfo) -> None:
        """Store the last depth camera info in a private attribut.

        Info:
            More info http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/CameraInfo.html

        Args:
            msg (CameraInfo): The message from the topic.
        """
        self._depth_camera_info = {
            'height': msg.height,
            'width': msg.width,
            'distortion_model': msg.distortion_model,
            'distortion_parameters': msg.D,
            'intrinsic_camera_matrix': msg.K,
            'rectification_matrix': msg.R,
            'extrinsic_camera_matrix': msg.P,
            'binning_x': msg.binning_x,
            'binning_y': msg.binning_y,
            'roi_x_offset': msg.roi.x_offset,
            'roi_y_offset': msg.roi.y_offset,
            'roi_height': msg.roi.height,
            'roi_width': msg.roi.width,
            'roi_do_rectify': msg.roi.do_rectify}

    def _depth_img_callback(self, msg: Image) -> None:
        """Store the last depth image in a private attribut.

        Args:
            msg (Image): The message from the topic.
        """
        flatten_depth_img = np.frombuffer(
            msg.data, dtype=np.uint16)  # shape =(width * height * 1)
        depth_img = flatten_depth_img.reshape(
            msg.height, msg.width)  # shape = (width, height, 2)
        self._depth_img = depth_img

    def _extrinsics_depth_to_color_callback(self, msg: Extrinsics) -> None:
        """Store the last extrinsics depth to color in a private attribut.

        Args:
            msg (Extrinsics): The message from the topic.
        """
        self._extrinsics_depth_to_color = {
            'rotation': msg.rotation,
            'translation': msg.translation}

    def _realsense2_camera_manager_bond_callback(self, msg: Status) -> None:
        """Store the last realsense2 camera manager bond in a private attribut.

        Args:
            msg (Status): The message from the topic.
        """
        self._realsense2_camera_manager_bond[msg.instance_id] = {
            'active': msg.active,
            'heartbeat_timeout': msg.heartbeat_timeout,
            'heartbeat_period': msg.heartbeat_period}
    # endregion

    def _show(self):
        cv2.namedWindow('realsense', cv2.WINDOW_AUTOSIZE)
        while True:
            cv2.waitKey(int(1000/self.display_fps))  # 25 fps
            # If user mannually close the window, stop the thread
            wnd_closed = not cv2.getWindowProperty(
                'realsense', cv2.WND_PROP_VISIBLE)
            if wnd_closed:
                self.display = False
                break
            # can be None if the first message is not received yet.
            if self._depth_img is None or self._color_img is None:
                continue
            # Apply colormap on depth image
            depth_colormap = self._depth_img_to_colormap(self._depth_img)
            images = np.hstack((self._color_img, depth_colormap))
            cv2.imshow('realsense', images)

    def _depth_img_to_colormap(self, depth_img: np.ndarray) -> np.ndarray:
        """Generate a rgb colormap from depth image.

        Args:
            depth_img (np.ndarray): Depth image. The shape is `(width, height)`.

        Returns:
            np.ndarray: Generated colormap. The shape is `(width, height, 3)`.
        """
        depth_img_clipped = np.clip(
            depth_img, self.display_depth_min, self.display_depth_max)
        depth_img_mapped = depth_img_clipped / \
            ((self.display_depth_max-self.display_depth_min)/257.0)
        depth_img8 = depth_img_mapped.astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_img8, cv2.COLORMAP_PLASMA)
        return depth_colormap

    # Getters
    # region
    def get_color_camera_info(self) -> dict:
        """Get camera info

        Returns:
            dict: Camera info
        """
        return copy.deepcopy(self._color_camera_info)

    def get_color_img(self) -> np.ndarray:
        """Return the last color image (rgb).

        Returns:
            np.ndarray: Last color image. The shape is `(width, height, 3)`.
        """
        return copy.deepcopy(self._color_img)

    def get_color_imgs(self) -> List[np.ndarray]:
        return list(self._color_img_buffer)

    def get_depth_camera_info(self) -> dict:
        """Get camera info

        Returns:
            dict: Camera info
        """
        return copy.deepcopy(self._depth_camera_info)

    def get_depth_img(self) -> np.ndarray:
        """Return the last depth images (left and right).

        Returns:
            np.ndarray: Last depth image. The shape is `(width, height)`.
        """
        return copy.deepcopy(self._depth_img)

    def get_extrinsics_depth_to_color(self) -> dict:
        """Get extrinsics depth to color.

        Returns:
            dict: Extrinsics depth to color.
        """
        return copy.deepcopy(self._extrinsics_depth_to_color)

    def get_realsense2_camera_manager_bond(self) -> dict:
        """Get extrinsics depth to color.

        Returns:
            dict: Extrinsics depth to color.
        """
        return copy.deepcopy(self._realsense2_camera_manager_bond)

    # endregion


if __name__ == '__main__':
    rospy.init_node('RealSenseInterfaceNode',
                    anonymous=True, disable_signals=True)
    r = RealSenseInterface(buffer_size=10, display=True, display_fps=10,
                           display_depth_min=30, display_depth_max=10000)
    import time
    time.sleep(3)
    print(len(r.get_color_imgs()))
