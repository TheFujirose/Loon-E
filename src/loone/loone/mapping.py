import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray, MultiArrayDimension
import numpy as np
import math

class Mapping(Node):
    def __init__(self):
        global_size = [50, 50]
        start_position = [4]

        super().__init__('Map_PubSub')
        self.global_pub = self.create_publisher(Int8MultiArray, 'global', 10)
        self.position_pub = self.create_publisher(Float32MultiArray, 'position', 10)
        self.objects_sub = self.create_subscription(Float32MultiArray, 'objects', self.object_callback(), 10)
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback(), 10)

        self.local_w = 20
        self.local_l = 20
        self.global_w = global_size[0]
        self.global_l = global_size[1]
        self.res = 0.5

        self.local_rows = self.get_cell(self.local_w)
        self.local_cols = self.get_cell(self.local_l)
        self.global_rows = self.get_cell(self.global_w)
        self.global_cols = self.get_cell(self.global_l)
        self.local_map = []
        self.global_map = np.zeros((self.global_rows, self.global_cols))

        match start_position:
            case 0.5: # Positive Y axis
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.5)]
            case 1: # Quadrant 1
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.75)]
            case 1.5: # Positive X axis
                self.global_position = [round(self.global_rows*0.5), round(self.global_cols*0.75)]
            case 2: # Quadrant 2
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.75)]
            case 2.5: # Negative Y axis
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.5)]
            case 3: # Quadrant 3
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.25)]
            case 3.5: # Negative X axis
                self.global_position = [round(self.global_rows*0.5), round(self.global_cols*0.25)]
            case 4: # Quadrant 4
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.25)]

        self.objects = None
        self.locations = None
        self.position = self.global_position
        self.position_0 = [None, None]
        self.heading = None

        self.publish_mapdata()

    #ROS - Publish
    def publish(self):
        msg = Int8MultiArray()
        msg.data = self.global_map
        self.global_pub.publish(msg)
        self.get_logger().info(f"Map: {msg.data}")

    def publish_position(self):
        msg = Float32MultiArray()
        msg.data = self.global_position
        self.position_pub.publish(msg)
        self.logger().info(f"Position: {msg.data}")
    
    def publish_mapdata(self):
        msg = Float32MultiArray()
        msg.data = [self.local_w, self.res]
        self.position_pub.publish(msg)
        self.get_logger().info(f"Map Data: {msg.data}")

    #General Code
    def get_cell(self, meter): #convert units in meters to units in cells
        cell = round(meter / self.res)

        return cell
    
    def get_map_cell(self, start, end): #Get heading from two coordinates
        #Possible edit: Use haversine formula instead
        lat = math.radians((start[1] + end[1]) / 2)
        y = self.get_cell((start[0] - end[0]) * 111000)
        x = self.get_cell((start[1] - end[1]) * 111000 * math.cos(lat))
        self.global_position[0] += y
        self.global_position[1] += x
    
    def get_global_map(self):
        self.get_map_cell(self.position_0, self.position)
        mid = round((self.global_cols - 1) / 2)
        for i in range(self.global_rows):
            for j in range(self.global_cols):
                if mid - i <= j <= mid + i: #Do not include excess (Camera has 90 degree FOV)
                    #Convert cartesian to polar
                    local_x = mid - j
                    local_y = i
                    r = math.sqrt(local_y**2 + local_x**2)
                    theta = math.atan(local_x, local_y)
                    if theta < 0: #Quadrant IV
                        theta += 2*math.pi()

                    #Find matching point in local map
                    theta += math.radians(self.heading)
                    global_x = round(r*math.sin(theta)) + self.global_position[0]
                    global_y = round(r*math.cos(theta)) + self.global_position[1]
                    self.global_map[global_y][global_x] = self.local_map[i][j]
        
        self.publish()

    def get_local_map(self):
        self.local_map = np.zeros((self.local_rows, self.local_cols))

        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            location = self.locations[i]

            match obstacle: #Values rounded up to nearest whole cell length, assuming resolution of 0.5 m
                case _:
                    obj_length = 0.5
                    obj_width = 0.5

            # Convert world units to grid cells
            objL = self.get_cell(obj_length)
            objW = self.get_cell(obj_width)

            objStartX = self.get_cell(location[0])
            objStartY = self.get_cell(location[1])

            # Write into the matrix
            for i in range(objL):
                for j in range(objW):
                    self.map[objStartY + j, objStartX + i] = obstacle
        
        if self.position is not None and self.heading is not None:
            self.get_global_map()
    
    #ROS - Subscribe
    def object_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Objects: {msg.data}")
        self.objects = data[0]
        self.locations = data[1]
        self.get_local_map()

    def phone_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Phone: {msg.data}")
        if data[0] != self.position_0[0] or data[1] != self.position_0[1]:
            self.position_0 = self.position
        self.position = [data[0], data[1], self.local_w, self.res]
        self.heading = data[3]
    
def main(args = None):
    rclpy.init(args = args)
    mapping = Mapping()
    try:
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        mapping.get_logger().info("Mapping node interrupted by user.")
    finally:
        mapping.shutdown()
        mapping.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
