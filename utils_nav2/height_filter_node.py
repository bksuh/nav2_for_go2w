#!/usr/bin/env python3
import math
from typing import Iterable, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

import tf2_ros
from tf2_ros import TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class HeightFilterNode(Node):
    def __init__(self):
        super().__init__('height_filter_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('input_topic', '/kiss/local_map')
        self.declare_parameter('output_topic', '/kiss/filtered_map')
        self.declare_parameter('ground_frame', 'odom')  # +Z가 '위'가 되도록 정의된 프레임
        self.declare_parameter('min_height', 0.650)                # ground_frame 기준 최소 높이 [m]
        self.declare_parameter('max_height', 1.4)        # 필요 시 상한, 미사용이면 inf
        self.declare_parameter('use_tf', True)                    # 입력 클라우드를 ground_frame으로 변환할지
        self.declare_parameter('preserve_stamp', True)            # 타임스탬프 유지 여부

        self.input_topic   = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic  = self.get_parameter('output_topic').get_parameter_value().string_value
        self.ground_frame  = self.get_parameter('ground_frame').get_parameter_value().string_value
        self.min_height    = float(self.get_parameter('min_height').get_parameter_value().double_value)
        self.max_height    = float(self.get_parameter('max_height').get_parameter_value().double_value)
        self.use_tf        = bool(self.get_parameter('use_tf').get_parameter_value().bool_value)
        self.preserve_stamp= bool(self.get_parameter('preserve_stamp').get_parameter_value().bool_value)

        # -----------------------------
        # TF buffer & listener
        # -----------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -----------------------------
        # QoS
        # -----------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,  # 'UNKNOWN'으로 나올 경우, 적절한 값(e.g., 10)을 지정하는 것이 일반적입니다.
            durability=DurabilityPolicy.VOLATILE
        )

        # -----------------------------
        # I/O
        # -----------------------------
        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_cb, sensor_qos
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, sensor_qos)

        self.get_logger().info(
            f"[height_filter_node] input='{self.input_topic}', output='{self.output_topic}', "
            f"ground_frame='{self.ground_frame}', min_height={self.min_height:.3f}, "
            f"max_height={'inf' if math.isinf(self.max_height) else f'{self.max_height:.3f}'}, "
            f"use_tf={self.use_tf}"
        )

    # -----------------------------
    # Callback
    # -----------------------------
    def cloud_cb(self, cloud: PointCloud2):
        # 1) 필요 시 ground_frame으로 변환
        cloud_in = cloud
        out_frame = self.ground_frame
        if self.use_tf:
            src_frame = cloud.header.frame_id
            if src_frame != self.ground_frame:
                try:
                    # 최신 TF로 변환 (cloud stamp 기준으로 lookup)
                    transform = self.tf_buffer.lookup_transform(
                        self.ground_frame, src_frame, rclpy.time.Time.from_msg(cloud.header.stamp),
                        timeout=rclpy.duration.Duration(seconds=0.2)
                    )
                    cloud_in = do_transform_cloud(cloud, transform)
                except TransformException as ex:
                    self.get_logger().warn(
                        f"TF transform failed: {src_frame} -> {self.ground_frame} ({ex})"
                    )
                    return
            else:
                out_frame = src_frame
        else:
            out_frame = cloud.header.frame_id

        # 2) XYZ만 추출 & NaN skip
        #    read_points returns generator over tuples (x,y,z)
        try:
            gen = pc2.read_points(cloud_in, field_names=('x', 'y', 'z'), skip_nans=True)
        except Exception as e:
            self.get_logger().error(f"read_points failed: {e}")
            return

        # 3) height(Z) 필터링
        min_h = self.min_height
        max_h = self.max_height
        filtered_pts: List[Tuple[float, float, float]] = []
        count_in = 0
        for p in gen:
            count_in += 1
            z = p[2]
            if z >= min_h and (math.isinf(max_h) or z <= max_h):
                filtered_pts.append((p[0], p[1], z))

        # 4) 출력 PointCloud2 (XYZ32)
        header = Header()
        header.frame_id = out_frame
        header.stamp = cloud_in.header.stamp if self.preserve_stamp else self.get_clock().now().to_msg()

        if len(filtered_pts) == 0:
            # 빈 클라우드라도 유효 헤더로 발행 (다운스트림 동기화용)
            empty_msg = pc2.create_cloud_xyz32(header, [])
            self.pub.publish(empty_msg)
            self.get_logger().debug(f"Published empty filtered cloud (in={count_in}, out=0)")
            return

        filtered_msg = pc2.create_cloud_xyz32(header, filtered_pts)
        self.pub.publish(filtered_msg)
        self.get_logger().debug(f"Published filtered cloud (in={count_in}, out={len(filtered_pts)})")


def main():
    rclpy.init()
    node = HeightFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
