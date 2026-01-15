import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('wrist_camera_static_tf_publisher')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        self.parent_frame = 'base_link'
        self.child_frame = 'camera_link'
        
        # Wait and fetch transform
        self.timer = self.create_timer(0.1, self.fetch_and_publish_transform)
        self.transform_published = False

    def fetch_and_publish_transform(self):
        if self.transform_published:
            return
            
        try:
            # Look up the transform
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Publish as static transform
            static_transform = TransformStamped()
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = self.parent_frame
            static_transform.child_frame_id = self.child_frame
            
            static_transform.transform.translation.x = transform.transform.translation.x
            static_transform.transform.translation.y = transform.transform.translation.y
            static_transform.transform.translation.z = transform.transform.translation.z
            static_transform.transform.rotation.x = transform.transform.rotation.x
            static_transform.transform.rotation.y = transform.transform.rotation.y
            static_transform.transform.rotation.z = transform.transform.rotation.z
            static_transform.transform.rotation.w = transform.transform.rotation.w
            
            self.broadcaster.sendTransform(static_transform)
            
            self.get_logger().info(f' - Captured and published static TF: {self.parent_frame} -> {self.child_frame}')
            self.get_logger().info(f' - Translation: [{transform.transform.translation.x:.3f}, '
                                 f'{transform.transform.translation.y:.3f}, '
                                 f'{transform.transform.translation.z:.3f}]')
            self.get_logger().info(f' - Rotation: [{transform.transform.rotation.x:.3f}, '
                                 f'{transform.transform.rotation.y:.3f}, '
                                 f'{transform.transform.rotation.z:.3f}, '
                                 f'{transform.transform.rotation.w:.3f}]')
            
            self.transform_published = True
            self.timer.cancel()
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info(f'Waiting for transform... ({e})')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()