import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Waypoint_node(Node): 
    def __init__(self):
        super().__init__('waypoint_node')
        self.vel_pub=self.create_publisher(Twist,'cmd_vel',10)
        self.timer=self.create_timer(0.01,self.on_timer)

    def on_timer(self):
        msg=Twist()
        msg.angular.z=0.1
        msg.linear.x=0.2
        self.vel_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    vel_node=Waypoint_node()

    try: 
        rclpy.spin(vel_node)

    except KeyboardInterrupt:
        pass 
    
    finally: 
        vel_node.destroy_node() 
        rclpy.shutdown()