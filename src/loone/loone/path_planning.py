import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import numpy as np

class Path(Node):
    def __init__(self):
        super().__init__('Path_PubSub')
        self.path_pub = self.create_publisher(Int8MultiArray, 'path', 10)
        self.map_sub = self.create_subscription(Int8MultiArray, 'global', self.map_callback(), 10)
        self.task_sub = self.create_subscription(Int8MultiArray, 'task_path', self.task_callback(), 10)

        self.path = [] #straight line path
        self.path_obstacles = [] #path avoiding obstacles
        self.waypoints = [] #self.path_obstacles with reduced resolution, based on self.dist
        self.dist = 4 #interval between waypoints, in cells
        self.radius = 2 #min distance from obstacles, in cells

        self.map = None
        self.x_start = None
        self.y_start = None
        self.x_end = None
        self.y_end = None

    #ROS - Publish
    def publish(self):
        msg = Int8MultiArray()
        msg.data = self.waypoints
        self.path_pub.publish(msg)
        self.get_logger().info(f"{msg.data}")

    #General Code
    def point_in_map(self, position):
        rows = len(self.map)
        cols = len(self.map[0])
        output = False
        
        if 0 <= position[0] < cols and 0 <= position[1] < rows: #If desired area to search is within map bounds
            output = True
        
        return output

    def find_obstacle(self, point):
        output = False
        y = point[0]
        x = point[1]
        
        for i in range (y - self.radius, y + self.radius + 1):
            for j in range (x - self.radius, x + self.radius + 1):
                position = (i, j)
                if self.point_in_map(position) and (self.map[i][j] != 0): #If obstacle in range
                    output = True
                    break
        
        return output

    def pathfind(self, start, expanded):
        parent_list = []
        
        if len(expanded)!=0: #If expanded is empty
            position=expanded[-1][1] #End node
            previous=expanded[-1][2] #Parent of end node
            while (previous!=start): #If parent is not start:
                parent_list.append(position) #Add node to path
                for point in expanded: #Find parent in expanded
                    if (point[1]==previous):
                        position=expanded[expanded.index(point)][1] #Parent node = new end node
                        previous=expanded[expanded.index(point)][2] #Parent of new end node
                        break        
            parent_list.append(position) #Add start node to path
            
        return parent_list

    def check_neighbours(self, start, end, position, unexpanded, expanded, checked):
        y = position[0]
        x = position[1]
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)] #Up, down, right, left, diagonal
        
        for i, j in dir:
            next = (y + i, x + j)
            if self.point_in_map(next) and not (next in checked or self.find_obstacle(next)): #If in map, not checked, and not near obstacle
                manhattan = abs(next[0] - end[0]) + abs(next[1] - end[1])
                cost = manhattan + len(self.pathfind(start, expanded))
                unexpanded.append([cost, next, position])
                checked.append(next)
            
    def update_waypoints(self, start, parent_list):
        location = self.path_obstacles.index(start) + 1 #Find point in path list
        
        for point in parent_list:
            self.path_obstacles.insert(location, point) #Add waypoints from obstacle avoidance
        self.path_obstacles.remove(parent_list[0])

    def avoid_obstacle(self, point):
        unexpanded=[] #Evaluated neighbours in form [cost, current position, previous position]
        expanded=[] #Visited in form [cost, current position, previous position]
        checked=[] #Checked cells in form [position]
        
        start = self.path[self.path.index(point) - 1]
        end_index = self.path.index(point)
        end = self.path[end_index]
        position = (start[0], start[1])

        while self.find_obstacle(end) and (end_index != len(self.path_obstacles) - 1): #move destination further away so it does not intersect with an obstacle
            self.path.remove(end)
            end = self.path[end_index]
        
        while position != end:
            self.check_neighbours(start, end, position, unexpanded, expanded, checked)
            
            if(unexpanded == []): #If no more nodes to search
                break

            next = min(unexpanded) #Select node with lowest cost
            expanded.append(next) #Move node to expanded list
            unexpanded.remove(next) #Remove node from unexpanded 
            position = expanded[-1][1] #Set to expand selected node on repeat

        parent_list = self.pathfind(start, expanded)
        self.update_waypoints(start, parent_list)
        
    def make_waypoints(self):
        self.path_obstacles = self.path

        for point in self.path:
            if self.find_obstacle(point): #If waypoint near obstacle, generate new path between previous and next waypoints
                self.avoid_obstacle(point)
                
        for point in self.path_obstacles:
            y = point[0]
            x = point[1]
            if ((self.path_obstacles.index(point) % self.dist == 0) or (point == self.path_obstacles[-1])): #Add every d points to waypoint list and last waypoint
                self.waypoints.append(point)

    def get_path(self):
        while self.find_obstacle((self.y_end, self.x_end)): #Ensure that end point is not near obstacle
            self.y_end = self.y_end - 1 #UPDATE
        
        dx = self.x_end - self.x_start #change in x
        dy = self.y_end - self.y_start #change in y

        if dx == 0: #vertical line
            for y in range(self.y_start, self.y_end + 1, np.sign(self.y_end - self.y_start)):
                self.path.append((y, self.x_start))
        elif dy == 0: #horizontal line
            for x in range(self.x_start, self.x_end, np.sign(self.x_end - self.x_start)):
                self.path.append((self.y_end, x))
        elif dy <= dx or -1 < dy / dx < 0: #Typical program
            m = dy / dx #Calculate slope
            b = self.y_start - m * self.x_start #Calculate y intercept
            for x in range(self.x_start, self.x_end + np.sign(dx), np.sign(dx)):
                y = round(m * x + b) #Find nearest y value
                self.path.append((y, x)) #Add point to path
        else: #Switch x and y so that m <= 1
            m = dx / dy #Calculate slope
            b = self.x_start - m * self.y_start #calculate x intercept
            for y in range(self.y_start, self.y_end + np.sign(dy), np.sign(dy)):
                x = round(m * y + b) #Find nearest x value
                self.path.append((y, x)) #Add point to path
        
        self.make_waypoints()
        self.publish()
    
    #ROS - Subscribe
    def map_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Map: {msg.data}")
        self.map = data

    def task_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Task: {msg.data}")
        self.y_start = data[0]
        self.x_start = data[1]
        self.y_end = data[2]
        self.x_end = data[3]
        if self.map is not None:
            self.get_path()
    
def main(args = None):
    rclpy.init(args = args)
    planning = Path()
    try:
        rclpy.spin(planning)
    except KeyboardInterrupt:
        planning.get_logger().info("Planning node interrupted by user.")
    finally:
        planning.shutdown()
        planning.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()