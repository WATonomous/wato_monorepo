# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""MapAnything as a ROS service for the eidos visual plugins.

Two services, both stateless (everything they need is in the request, the camera calibration topics
and TF):

* ``EstimateRelativePose``: the motion of the camera rig between two sets of images, from one
  MapAnything pass over all of them.
* ``ComputePlaceDescriptors``: one MegaLoc descriptor per image, for finding which places look
  alike.

The services appear once the models are loaded, so clients can use service availability as the
"ready" signal.
"""

import os
import threading
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from eidos_msgs.srv import ComputePlaceDescriptors, EstimateRelativePose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener

from map_anything.cloud import pack_xyzrgb, rig_cloud
from map_anything.depth import sparse_depth, xyz_from_cloud
from map_anything.model import DEFAULT_CHECKPOINT, View, centred_view
from map_anything.rig import relative_rig_pose, relative_rig_pose_per_camera
from visual_odometry.geometry import split_transform
from visual_odometry.pipeline import load_extrinsics, resolve_path
from visual_odometry.ros_io import transform_msg_to_matrix

FITS = ("per_camera", "joint")


class RequestError(Exception):
    """A request that cannot be answered; the text goes back to the client."""


def cloud_msg(stamp, frame: str, points: np.ndarray, colors: np.ndarray) -> PointCloud2:
    """PointCloud2 with float32 x, y, z and rgb (packed in a float32, as RViz, Foxglove and PCL read it)."""
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, name in enumerate(("x", "y", "z", "rgb"))
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * len(points)
    msg.data = pack_xyzrgb(points, colors)
    msg.is_dense = True
    return msg


class MapAnythingServer(Node):
    def __init__(self):
        super().__init__("map_anything")
        self.checkpoint = self.declare_parameter("checkpoint", DEFAULT_CHECKPOINT).value
        self.device = self.declare_parameter("device", "cuda").value
        # weights are several GB: keep them on the mounted models volume, not in the container
        self.cache_dir = self.declare_parameter(
            "cache_dir", "/opt/watonomous/models/map_anything"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/{camera}/camera_info"
        ).value
        self.declare_parameter("cameras", Parameter.Type.STRING_ARRAY)
        self.crop_bottom = int(self.declare_parameter("crop_bottom", 0).value)
        self.fit = self.declare_parameter("fit", "per_camera").value
        self.place_recognition = bool(
            self.declare_parameter("place_recognition", True).value
        )
        extrinsics_file = self.declare_parameter("extrinsics_file", "").value
        self.extrinsics_rig_frame = self.declare_parameter(
            "extrinsics_rig_frame", "base_link"
        ).value
        if self.fit not in FITS:
            raise ValueError(f"fit must be one of {FITS}, not {self.fit!r}")

        extrinsics_file = resolve_path(
            extrinsics_file,
            os.path.join(get_package_share_directory("visual_odometry"), "config"),
        )
        # {camera: T_extrinsics_rig_frame_camera}, replacing TF for those cameras
        self.overrides = load_extrinsics(extrinsics_file, self.extrinsics_rig_frame)

        self._lock = threading.Lock()
        self._infos: Dict[str, CameraInfo] = {}
        self._info_subs = {}
        self._extrinsics: Dict[tuple, np.ndarray] = {}
        self._io_group = ReentrantCallbackGroup()
        self._gpu_group = MutuallyExclusiveCallbackGroup()  # one inference at a time
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        try:
            for camera in self.get_parameter("cameras").value or []:
                self._watch(camera)
        except rclpy.exceptions.ParameterUninitializedException:
            pass  # cameras are then subscribed to on first use

        os.makedirs(self.cache_dir, exist_ok=True)
        os.environ.setdefault("HF_HOME", os.path.join(self.cache_dir, "huggingface"))
        os.environ.setdefault("TORCH_HOME", os.path.join(self.cache_dir, "torch"))
        self.model = None
        self.place = None
        self._loader = threading.Thread(target=self._load, daemon=True)
        self._loader.start()

    # ---- startup --------------------------------------------------------------------------------

    def _load(self) -> None:
        from map_anything.model import MapAnythingModel  # imports torch

        start = time.perf_counter()
        self.get_logger().info(
            f"loading {self.checkpoint} on {self.device} (cache {self.cache_dir})"
        )
        self.model = MapAnythingModel(self.checkpoint, self.device)
        if self.place_recognition:
            from map_anything.place import PlaceDescriptor

            self.place = PlaceDescriptor(self.device)
            self.create_service(
                ComputePlaceDescriptors,
                "~/compute_place_descriptors",
                self._on_descriptors,
                callback_group=self._gpu_group,
            )
        self.create_service(
            EstimateRelativePose,
            "~/estimate_relative_pose",
            self._on_relative_pose,
            callback_group=self._gpu_group,
        )
        self.get_logger().info(
            f"ready after {time.perf_counter() - start:.0f} s: fit={self.fit}, "
            f"crop_bottom={self.crop_bottom}, extrinsics="
            f"{'TF + ' + ', '.join(self.overrides) if self.overrides else 'TF'}"
        )

    def _watch(self, camera: str) -> None:
        """Subscribe to a camera's CameraInfo (once)."""
        with self._lock:
            if camera in self._info_subs:
                return
            self._info_subs[camera] = self.create_subscription(
                CameraInfo,
                self.camera_info_topic.format(camera=camera),
                lambda msg, camera=camera: self._infos.setdefault(camera, msg),
                qos_profile_sensor_data,
                callback_group=self._io_group,
            )

    # ---- calibration ----------------------------------------------------------------------------

    def _intrinsics(self, camera: str) -> np.ndarray:
        info = self._infos.get(camera)
        if info is None:
            self._watch(camera)
            raise RequestError(
                f"no CameraInfo yet on {self.camera_info_topic.format(camera=camera)}"
            )
        # P: the images are rectified
        return np.array(info.p, dtype=float).reshape(3, 4)[:, :3]

    def _lookup(self, target: str, source: str) -> np.ndarray:
        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except TransformException as error:
            raise RequestError(f"no transform {target} <- {source}: {error}") from error
        return transform_msg_to_matrix(tf.transform)

    def _rig_from_camera(self, rig_frame: str, camera: str) -> np.ndarray:
        key = (rig_frame, camera)
        if key not in self._extrinsics:
            if camera in self.overrides:
                extrinsic = self.overrides[camera]
                if rig_frame != self.extrinsics_rig_frame:
                    extrinsic = (
                        self._lookup(rig_frame, self.extrinsics_rig_frame) @ extrinsic
                    )
            else:
                extrinsic = self._lookup(rig_frame, camera)
            self._extrinsics[key] = extrinsic
        return self._extrinsics[key]

    # ---- services -------------------------------------------------------------------------------

    @staticmethod
    def _decode(image) -> np.ndarray:
        bgr = cv2.imdecode(
            np.frombuffer(bytes(image.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if bgr is None:
            raise RequestError(f"cannot decode the image of {image.header.frame_id}")
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def _views(self, images, depths=()) -> List[View]:
        if depths and len(depths) != len(images):
            raise RequestError(f"{len(depths)} depth clouds for {len(images)} images")
        views = []
        for i, image in enumerate(images):
            view = centred_view(
                self._decode(image),
                self._intrinsics(image.header.frame_id),
                self.crop_bottom,
            )
            if depths and depths[i].width * depths[i].height > 0:
                cloud = depths[i]
                offsets = {f.name: f.offset for f in cloud.fields}
                if not {"x", "y", "z"} <= offsets.keys():
                    raise RequestError("depth clouds need x, y, z fields")
                points = xyz_from_cloud(
                    bytes(cloud.data),
                    cloud.point_step,
                    cloud.width * cloud.height,
                    (offsets["x"], offsets["y"], offsets["z"]),
                )
                # The crop only moves the principal point: camera-frame points project with the
                # cropped intrinsics straight into the cropped image.
                view.depth = sparse_depth(points, view.intrinsics, view.rgb.shape[:2])
            views.append(view)
        return views

    def _on_relative_pose(self, request, response):
        try:
            cameras_a = [image.header.frame_id for image in request.images_a]
            cameras_b = [image.header.frame_id for image in request.images_b]
            if not cameras_a or not cameras_b:
                raise RequestError("both instants need at least one image")
            rig_frame = request.rig_frame or self.extrinsics_rig_frame
            extrinsics_a = [self._rig_from_camera(rig_frame, c) for c in cameras_a]
            extrinsics_b = [self._rig_from_camera(rig_frame, c) for c in cameras_b]
            views = self._views(request.images_a, request.depth_a) + self._views(
                request.images_b, request.depth_b
            )
            start = time.perf_counter()
            if request.return_cloud:
                dense = self.model.reconstruct(views)
                world_from_cameras = [v.world_from_camera for v in dense]
                confidence = float(np.mean([v.confidence.mean() for v in dense]))
            else:
                prediction = self.model.predict(views)
                world_from_cameras = prediction.world_from_cameras
                confidence = float(np.mean(prediction.confidences))
            infer_ms = (time.perf_counter() - start) * 1e3
            poses_a = world_from_cameras[: len(cameras_a)]
            poses_b = world_from_cameras[len(cameras_a) :]
            if request.return_cloud:
                dense_b = dense[len(cameras_a) :]
                points, colors = rig_cloud(
                    [v.points for v in dense_b],
                    [v.valid for v in dense_b],
                    [v.rgb for v in dense_b],
                    poses_b,
                    extrinsics_b,
                    max_range=request.cloud_max_range,
                    voxel=request.cloud_voxel,
                )
                response.cloud = cloud_msg(
                    request.images_b[0].header.stamp, rig_frame, points, colors
                )
            if self.fit == "per_camera" and cameras_a == cameras_b:
                estimate = relative_rig_pose_per_camera(poses_a, poses_b, extrinsics_a)
            else:
                estimate = relative_rig_pose(
                    poses_a, extrinsics_a, poses_b, extrinsics_b
                )
        except RequestError as error:
            response.success = False
            response.message = str(error)
            return response
        except Exception as error:  # noqa: BLE001 - a failed request must not take the server down
            self.get_logger().error(f"relative pose request failed: {error!r}")
            response.success = False
            response.message = repr(error)
            return response

        translation, quat = split_transform(estimate.rig_a_from_rig_b)
        position, orientation = (
            response.relative_pose.position,
            response.relative_pose.orientation,
        )
        position.x, position.y, position.z = translation.tolist()
        orientation.x, orientation.y, orientation.z, orientation.w = quat.tolist()
        response.success = True
        response.trans_spread = estimate.trans_spread
        response.rot_spread = estimate.rot_spread
        response.confidence = confidence
        response.cameras = estimate.cameras
        response.inference_ms = float(infer_ms)
        return response

    def _on_descriptors(self, request, response):
        try:
            if not request.images:
                raise RequestError("no images")
            descriptors = self.place.describe(
                [self._decode(image) for image in request.images]
            )
        except RequestError as error:
            response.success = False
            response.message = str(error)
            return response
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"descriptor request failed: {error!r}")
            response.success = False
            response.message = repr(error)
            return response
        response.success = True
        response.dim = int(descriptors.shape[1])
        response.descriptors = descriptors.reshape(-1).tolist()
        return response


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MapAnythingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
