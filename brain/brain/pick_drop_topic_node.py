
import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Bool, String
from grumpy_interfaces.srv import PickAndDropObject


class PickDropTopicNode(Node):
    
    def __init__(self):
        super().__init__('pick_drop_topic_node') 

        self.pick_drop_status_pub = self.create_publisher(String, 'brain/pick_drop_status', 1)

        self.pick_node_call = rclpy.create_node('brain_pick_call_node')  # Create a node for the position service
        self.drop_node_call = rclpy.create_node('brain_drop_call_node')  # Create a node for the arm camera service

        self.pick_client = self.pick_node_call.create_client(PickAndDropObject, '/arm_services/pick_object')
        self.drop_client = self.drop_node_call.create_client(PickAndDropObject, '/arm_services/drop_object')

        while not self.pick_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/pick_object...')

        while not self.drop_client.wait_for_service(timeout_sec=1.0):
            self._logger.info('Waiting for /arm_services/drop_object...')

        self.create_subscription(String, 'brain/action/pick_drop', self.pick_drop_cb, 1)
    
    def pick_drop_cb(self, msg:String):

        req = PickAndDropObject.Request()

        if msg.data == 'Pick':
            self.get_logger().info('Calling Pick service')
            future = self.pick_client.call_async(req)
            self.get_logger().info('Got future')
            rclpy.spin_until_future_complete(self.pick_node_call, future)
            res = future.result()
        else:
            self.get_logger().info('Calling Drop service')
            future = self.drop_client.call_async(req)
            self.get_logger().info('Got future')
            rclpy.spin_until_future_complete(self.drop_node_call, future)
            res = future.result()
        
        self.get_logger().info('Got result')

        if res.success:
            self.get_logger().info('Pick/Drop successful, publishing to brain')
            msg = String()
            msg.data = 'Success'
            self.pick_drop_status_pub.publish(msg)
        else:
            self.get_logger().info('Pick/Drop failure, publishing to brain')
            msg = String()
            msg.data = 'Failure'
            self.pick_drop_status_pub.publish(msg)
            

        
def main():
    rclpy.init()
    node = PickDropTopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()



