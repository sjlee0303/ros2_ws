#!/usr/bin/env python3
# cone_fusion_node.py – DetectionArray × LiDAR centroid 매칭
# ===========================================================
# YOLO bbox 와 LiDAR centroid 를 시간 동기 후,
# bbox 내부에 투영되는 LiDAR 점에 색을 입혀
#   PointCloud2 + MarkerArray 발행
# -----------------------------------------------------------

# ────────────────────────────────────────────────────────────
# ▼▼▼ 튜닝 파라미터 ▼▼▼
DET_TOPIC      = '/detections/front_down'        # YOLO 결과
LIDAR_TOPIC    = '/cluster_centroids'            # LiDAR centroid
LIDAR_FRAME    = 'velodyne'
CAMERA_FRAME   = 'camera_front_down_frame'

CLOUD_OUT      = '/cones/colored_centroids'
MARKER_OUT     = '/cones/markers'

FX, FY = 533.32761, 533.25104                    # 카메라 내부 파라미터
CX, CY = 316.61534, 248.71553

MARGIN_PX      = 5       # bbox 경계 여유 (픽셀)
MARKER_SIZE    = 0.25    # Sphare 구 지름 [m]
MARKER_LIFETIME_SEC = 0  # 0 = 영구
# ▲▲▲ 튜닝 파라미터 ▲▲▲
# ────────────────────────────────────────────────────────────


import rclpy, math, struct, numpy as np
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data
from message_filters import Subscriber, ApproximateTimeSynchronizer

# ROS 메시지
from interfaces_pkg.msg import DetectionArray
from sensor_msgs.msg     import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg      import PointStamped, Vector3
from builtin_interfaces.msg import Duration

# TF
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


def pc2_to_xyz_array(msg: PointCloud2) -> np.ndarray:
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
    pts  = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
    return np.vstack([pts['x'], pts['y'], pts['z']]).T.astype(np.float64)


class BBoxFusionNode(Node):
    def __init__(self):
        super().__init__('bbox_fusion_node')

        # TF buffer
        self.tf_buf = Buffer()
        self.tf_lst = TransformListener(self.tf_buf, self)

        # Pubs
        self.pub_pc = self.create_publisher(PointCloud2, CLOUD_OUT, 10)
        self.pub_mk = self.create_publisher(MarkerArray, MARKER_OUT, 10)

        # Subs + 동기
        from rclpy.qos import qos_profile_sensor_data  # 이미 위에 있음

        sub_det = Subscriber(self, DetectionArray, DET_TOPIC,
                            qos_profile=qos_profile_sensor_data)
        sub_lid = Subscriber(self, PointCloud2,   LIDAR_TOPIC,
                            qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([sub_det, sub_lid],
                                                queue_size=10, slop=0.05)
        self.sync.registerCallback(self.cb_pair)

        self._prev_cloud: PointCloud2 | None = None   # 깜빡임 방지 캐시
        self.get_logger().info(f'Ready  (sync: {DET_TOPIC} + {LIDAR_TOPIC})')

    # ─── Synced callback ──────────────────────────────────
    def cb_pair(self, det: DetectionArray, lid: PointCloud2):
        # TF 준비?
        if not self.tf_buf.can_transform(
                CAMERA_FRAME, LIDAR_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)):
            self.get_logger().warn_throttle(5000, 'TF not ready')
            return
        try:
            T = self.tf_buf.lookup_transform(
                CAMERA_FRAME, LIDAR_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF fail: {e}'); return

        centers = pc2_to_xyz_array(lid)
        out_pts, markers = [], MarkerArray()

        for idx, d in enumerate(det.detections):
            bb   = d.bbox
            cx, cy = bb.center.position.x, bb.center.position.y
            half_w, half_h = bb.size.x*0.5, bb.size.y*0.5
            rgb = (255,255,0) if 'Y' in d.class_name else (0,0,255)

            best, pt_best = float('inf'), None
            for x,y,z in centers:
                pt_in = PointStamped(header=lid.header)
                pt_in.point.x, pt_in.point.y, pt_in.point.z = x,y,z
                pt_cam = tf2_geometry_msgs.do_transform_point(pt_in, T)
                # print(f"[DEBUG] pt_cam (Camera frame): x={pt_cam.point.x:.2f}, y={pt_cam.point.y:.2f}, z={pt_cam.point.z:.2f}")
                
                if pt_cam.point.z < 0.05:        # 카메라 뒤
                    continue
                u = FX*pt_cam.point.x/pt_cam.point.z + CX
                v = FY*pt_cam.point.y/pt_cam.point.z + CY
                print(f"[DEBUG] u,v : u={u}, v={v}")
                if abs(cx-u) > half_w+MARGIN_PX or abs(cy-v) > half_h+MARGIN_PX:
                    continue
                dist = math.hypot(cx-u, cy-v)
                if dist < best:
                    best, pt_best = dist, (x,y,z)

            if pt_best is None:
                continue

            out_pts.append((*pt_best, *rgb))

            m = Marker(header=lid.header, ns='cones', id=idx, type=Marker.SPHERE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pt_best
            m.scale = Vector3(x=MARKER_SIZE, y=MARKER_SIZE, z=MARKER_SIZE)
            r,g,b = [c/255 for c in rgb]; m.color.r, m.color.g, m.color.b, m.color.a = r,g,b,1.0
            m.lifetime = Duration(sec=int(MARKER_LIFETIME_SEC),
                                  nanosec=int((MARKER_LIFETIME_SEC%1)*1e9))
            markers.markers.append(m)

        if out_pts:
            cloud = self._mk_cloud(out_pts, lid.header)
            self.pub_pc.publish(cloud);  self._prev_cloud = cloud
            self.pub_mk.publish(markers)
        elif self._prev_cloud:
            # 새 매칭 실패 → 직전 클라우드 재전송 (RViz 깜빡 방지)
            self.pub_pc.publish(self._prev_cloud)

    # ─── Helper: build PointCloud2 ─────────────────────────
    @staticmethod
    def _mk_cloud(pts_rgb, hdr) -> PointCloud2:
        from sensor_msgs_py import point_cloud2 as pc2
        fields = [PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                  PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                  PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)]
        pack = lambda r,g,b: struct.unpack('f', struct.pack('I',(r<<16)|(g<<8)|b))[0]
        pts  = [(x,y,z,pack(r,g,b)) for x,y,z,r,g,b in pts_rgb]
        return pc2.create_cloud(hdr, fields, pts)


def main():
    rclpy.init(); node=BBoxFusionNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()