#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer

import numpy as np
import struct
import cv2
from collections import deque

from sensor_msgs.msg import PointCloud2, PointField, Image
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3, PointStamped
from builtin_interfaces.msg import Duration

from interfaces_pkg.msg import DetectionArray

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from sensor_msgs_py import point_cloud2 as pc2
from cv_bridge import CvBridge

LIDAR_FRAME = 'velodyne'  # LiDAR frame ID (고정)
MARGIN_PX = 5
MARKER_SIZE = 0.25
MARKER_LIFETIME_SEC = 0  # 0 = 영구


def pointcloud2_to_xyz_array(msg: PointCloud2):
    points = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([p[0], p[1], p[2]])
    return np.array(points, dtype=np.float64)


def project_points(points_3d, P):
    n = points_3d.shape[0]
    homo_pts = np.hstack((points_3d, np.ones((n, 1))))
    pts_2d_hom = P @ homo_pts.T
    pts_2d = pts_2d_hom[:2, :] / pts_2d_hom[2, :]
    return pts_2d.T


def sigma_rule_filter(distances, sigma=2.0):
    if len(distances) == 0:
        return []
    distances = np.array(distances)
    mean = np.mean(distances)
    std = np.std(distances)
    filtered = distances[(distances >= mean - sigma * std) & (distances <= mean + sigma * std)]
    return filtered.tolist()


def detect_cone_color(image, bbox):
    """HSV 기반 색상 판별"""
    cx, cy = int(bbox.center.position.x), int(bbox.center.position.y)
    w, h = int(bbox.size.x), int(bbox.size.y)

    x1, y1 = max(cx - w // 2, 0), max(cy - h // 2, 0)
    x2, y2 = min(cx + w // 2, image.shape[1]), min(cy + h // 2, image.shape[0])

    roi = image[y1:y2, x1:x2]
    if roi.size == 0:
        return "unknown", (255, 255, 255)  # 디폴트 흰색

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue_mean = np.mean(hsv[:, :, 0])

    if 15 <= hue_mean <= 35:  # 노란색 범위
        return "yellow", (255, 255, 0)
    elif 0 <= hue_mean <= 10 or 160 <= hue_mean <= 180:  # 빨간색 범위
        return "red", (255, 0, 0)
    else:
        return "unknown", (200, 200, 200)  # 기타 색상


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        # 파라미터 선언
        self.declare_parameter('det_topic', '/detections/front_down')
        self.declare_parameter('lidar_topic', '/lidar_pre_down')
        self.declare_parameter('camera_frame', 'camera_front_down_frame')
        #self.declare_parameter('camera_image_topic', '/camera_front_down/image_raw')
        self.declare_parameter('cloud_out_cone', '/cones/colored_points_cone')
        self.declare_parameter('cloud_out_drum', '/cones/colored_points_drum')
        self.declare_parameter('marker_out', '/cones/markers')
        self.declare_parameter('projection_matrix', [533.32761, 0.0, 316.61534, 0.0,
                                                     0.0, 533.25104, 248.71553, 0.0,
                                                     0.0, 0.0, 1.0, 0.0])

        det_topic = self.get_parameter('det_topic').get_parameter_value().string_value
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        #image_topic = self.get_parameter('camera_image_topic').get_parameter_value().string_value
        self.cloud_out_cone = self.get_parameter('cloud_out_cone').get_parameter_value().string_value
        self.cloud_out_drum = self.get_parameter('cloud_out_drum').get_parameter_value().string_value
        self.marker_out = self.get_parameter('marker_out').get_parameter_value().string_value

        P_list = self.get_parameter('projection_matrix').get_parameter_value().double_array_value
        self.P = np.array(P_list, dtype=np.float64).reshape(3, 4)

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_cloud_cone = self.create_publisher(PointCloud2, self.cloud_out_cone, 10)
        self.pub_cloud_drum = self.create_publisher(PointCloud2, self.cloud_out_drum, 10)
        self.pub_marker = self.create_publisher(MarkerArray, self.marker_out, 10)

        # Subscribers
        sub_det = Subscriber(self, DetectionArray, det_topic, qos_profile=qos_profile_sensor_data)
        sub_lid = Subscriber(self, PointCloud2, lidar_topic, qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([sub_det, sub_lid], queue_size=10, slop=0.05)
        self.sync.registerCallback(self.callback)

        # 이미지 구독 추가
        self.bridge = CvBridge()
        self.image_queue = deque(maxlen=5)
        #self.create_subscription(Image, image_topic, self.image_cb, qos_profile_sensor_data)

        self.prev_cloud_cone = None
        self.prev_cloud_drum = None

        self.get_logger().info(f'Fusion node ready. Syncing {det_topic} + {lidar_topic} with frame {self.camera_frame}')

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_queue.append((msg.header.stamp, cv_img))

    def get_latest_image(self, stamp):
        """Detection timestamp에 가장 가까운 이미지 반환"""
        if not self.image_queue:
            return None
        return min(self.image_queue, key=lambda x: abs((stamp.sec + stamp.nanosec * 1e-9) - (x[0].sec + x[0].nanosec * 1e-9)))[1]

    def callback(self, det_msg, lidar_msg):
        # TF 변환 확인
        if not self.tf_buffer.can_transform(
            self.camera_frame, LIDAR_FRAME, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)):
            self.get_logger().warn_throttle(1000, "TF not ready")
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.camera_frame, LIDAR_FRAME, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        # LiDAR 포인트 Nx3 변환
        points = pointcloud2_to_xyz_array(lidar_msg)
        if points.shape[0] == 0:
            return

        out_points_cone = []
        out_points_drum = []
        markers = MarkerArray()

        # LiDAR 포인트 → 카메라 프레임 변환
        pts_cam = []
        for pt in points:
            pt_stamped = PointStamped()
            pt_stamped.header = lidar_msg.header
            pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = pt
            pt_cam = tf2_geometry_msgs.do_transform_point(pt_stamped, trans)
            pts_cam.append([pt_cam.point.x, pt_cam.point.y, pt_cam.point.z])
        pts_cam = np.array(pts_cam)
        pts_2d = project_points(pts_cam, self.P)

        # Detection과 매칭 + HSV 색 판별
        latest_img = self.get_latest_image(det_msg.header.stamp)
        if latest_img is None:
            self.get_logger().warn("No recent camera image available for HSV color detection")
            return

        for idx, det in enumerate(det_msg.detections):
            bb = det.bbox
            cx, cy = bb.center.position.x, bb.center.position.y
            half_w, half_h = bb.size.x * 0.5, bb.size.y * 0.5

            mask_x = (pts_2d[:, 0] >= (cx - half_w - MARGIN_PX)) & (pts_2d[:, 0] <= (cx + half_w + MARGIN_PX))
            mask_y = (pts_2d[:, 1] >= (cy - half_h - MARGIN_PX)) & (pts_2d[:, 1] <= (cy + half_h + MARGIN_PX))
            mask = mask_x & mask_y

            points_in_bbox = pts_cam[mask]
            dists = points_in_bbox[:, 2] if points_in_bbox.shape[0] > 0 else np.array([])
            filtered_dists = sigma_rule_filter(dists, sigma=2.0)

            if len(filtered_dists) == 0:
                continue

            rep_dist = np.median(filtered_dists)
            rep_idx = np.argmin(np.abs(points_in_bbox[:, 2] - rep_dist))
            rep_point = points_in_bbox[rep_idx]

            # 카메라 → LiDAR 역변환
            try:
                trans_inv = self.tf_buffer.lookup_transform(
                    LIDAR_FRAME, self.camera_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                rep_point_st = PointStamped()
                rep_point_st.header.frame_id = self.camera_frame
                rep_point_st.point.x, rep_point_st.point.y, rep_point_st.point.z = rep_point
                rep_point_lidar = tf2_geometry_msgs.do_transform_point(rep_point_st, trans_inv)
                x, y, z = rep_point_lidar.point.x, rep_point_lidar.point.y, rep_point_lidar.point.z
            except Exception:
                continue

            # HSV 기반 색 판별 (YOLO class_name이 cone인 경우)
            if det.class_name.lower() == "cone":
                color_name, rgb = detect_cone_color(latest_img, bb)
                out_points_cone.append((x, y, z, *rgb))
            elif det.class_name.lower() == "drum":
                # 드럼에 대해 색상 임의 지정 (노란색)
                rgb = (255, 255, 0)
                out_points_drum.append((x, y, z, *rgb))
            else:
                # cone, drum 아니면 드럼 그룹에 넣거나 무시 가능 (여기선 무시)
                continue

            # Marker 생성 (cone과 drum 모두 같은 마커 토픽에 발행)
            m = Marker()
            m.header = lidar_msg.header
            m.ns = 'cones_and_drums'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.scale = Vector3(x=MARKER_SIZE, y=MARKER_SIZE, z=MARKER_SIZE)
            r, g, b = [c / 255.0 for c in rgb]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.lifetime = Duration(sec=int(MARKER_LIFETIME_SEC))
            markers.markers.append(m)

        # 퍼블리시: cone, drum 포인트클라우드 각각
        if out_points_cone:
            cloud_msg_cone = self.make_cloud(out_points_cone, lidar_msg.header)
            self.pub_cloud_cone.publish(cloud_msg_cone)
            self.prev_cloud_cone = cloud_msg_cone
        elif self.prev_cloud_cone:
            self.pub_cloud_cone.publish(self.prev_cloud_cone)

        if out_points_drum:
            cloud_msg_drum = self.make_cloud(out_points_drum, lidar_msg.header)
            self.pub_cloud_drum.publish(cloud_msg_drum)
            self.prev_cloud_drum = cloud_msg_drum
        elif self.prev_cloud_drum:
            self.pub_cloud_drum.publish(self.prev_cloud_drum)

        # 마커는 통합 토픽으로 발행
        if markers.markers:
            self.pub_marker.publish(markers)

    def make_cloud(self, points_rgb, header):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        def pack_rgb(r, g, b):
            # RGB 8bit to float32 packed
            return struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]

        pts = [(x, y, z, pack_rgb(r, g, b)) for x, y, z, r, g, b in points_rgb]
        return pc2.create_cloud(header, fields, pts)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
