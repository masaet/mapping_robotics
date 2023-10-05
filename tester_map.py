import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav2_msgs.msg import BehaviorTreeLog

import numpy as np



class mapper(Node):
    def __init__(self):
        super().__init__('tester_map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        
    def map_callback(self, msg:OccupancyGrid):
        #Creates grid from topic:/map 
        data = np.array(msg.data)
        grid = np.reshape(data, (-1, msg.info.height))
        self.get_logger().info('grid:{}'.format(grid.shape))

        
        res = msg.info.resolution   #resolution of grid
        origin = msg.info.origin    #gridpoint (0,0) in x,y Pose coordinates 


        #_____________CHATGPT(5/10 ROS2 Robot Pose Sub)_______________
        #Fetch Turtlebots position in Pose coordinates
        trans = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        x, y = trans.transform.translation.x, trans.transform.translation.y
        
        #Converts Pose cordinates to grid cordinates
        grid_x = int((x - origin.position.x) / res)
        grid_y = int((y - origin.position.y) / res)

        self.get_logger().info('x:{}, y:{}'.format(grid_x,grid_y))
        

        #converts grid cordinates to pose cordinates (grid is the written integers)
        goal_x = 40 * res + origin.position.x
        goal_y = 20 * res + origin.position.y

        self.get_logger().info('goal_pos:x{} y{}'.format(goal_x,goal_y))

        #__________________________________________________________________________





         



        
def main(args=None):
    rclpy.init(args=args)
    map_cmder = mapper()
    rclpy.spin(map_cmder)


if __name__ == '__main__':
    main()
