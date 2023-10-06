#!/usr/bin/env python3 
import cv2
import numpy as np
from queue import PriorityQueue
from chase.msg import ps
import rospy
import time
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion,quaternion_from_euler
# from my_robot_controller.msg import pos

cam_h=12
star=time.time()
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
gap=1
start = None
end=None
width=800
cg=0
rg=0
rollg=0.0
pitchg=0.0
yawg=0.0
ci=0
ri=0
rolli=0.0
pitchi=0.0
yawi=0.0
pub=rospy.Publisher("path_data",ps,queue_size=10)
img=cv2.imread("/home/jainam/catkin_ws/src/test3/scripts/images/img_f1.jpg")
img=cv2.resize(img,(800,800))
img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
img=cv2.flip(img, 1)
class Spot:
    def __init__(self,row,col,width,total_rows):
        global img
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.b,self.g,self.r=img[self.x,self.y]
        self.color=(int(self.b),int(self.g),int(self.r))  
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col
    
    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == ORANGE

    def is_end(self):
        return self.color == TURQUOISE

    def make_start(self):
        self.color = ORANGE
        self.draw()

    def make_closed(self):
        self.color = RED
        self.draw()

    def make_open(self):
        self.color = GREEN
        self.draw()

    def make_end(self):
        self.color = TURQUOISE
        self.draw()

    def make_path(self):
        self.color = PURPLE
        self.draw()
    
    def make_wp(self):
        self.color = BLUE
        self.draw()

    def draw(self):
         global img
         cv2.rectangle(img, (self.x,self.y+self.width), (self.x+self.width,self.y), self.color,-1)

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        return False

def pose_callbackg(pose:Odometry):
    global rg,cg,rollg,pitchg,yawg,cam_h
    orientation_q=pose.pose.pose.orientation
    orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (rollg,pitchg,yawg)=euler_from_quaternion(orientation_list)
    xg=pose.pose.pose.position.x
    yg=pose.pose.pose.position.y
    t=50/(1.6*cam_h)
    cg=int(t*xg+25.5)
    rg=int(t*yg+25.5)
    if(rg>=49):
        ri=48
    if(cg>=49):
        ri=48

def pose_callbacki(pose:Odometry):
    global ci,ri,rolli,pitchi,yawi,cam_h
    orientation_q=pose.pose.pose.orientation
    orientation_list=[orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
    (rolli,pitchi,yawi)=euler_from_quaternion(orientation_list)
    xi=pose.pose.pose.position.x
    yi=pose.pose.pose.position.y
    t=50/(1.6*cam_h)
    ci=int(t*xi+25.5)
    ri=int(t*yi+25.5)
    if(ri>=49):
        ri=48                
    if(ci>=49):
        ri=48    
# def pose(pose:pos):
#     global ci,ri,cam_h,cg,rg
#     xi=pose.xp
#     yi=pose.yp
#     t=50/(1.6*cam_h)
#     ci=int(t*xi+25.5)
#     ri=int(t*yi+25.5)
#     xg=pose.xt
#     yg=pose.yt
#     cg=int(t*xg+25.5)
#     rg=int(t*yg+25.5)

def h(p1, p2):
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)

def publish(point):
        global pub
        global cam_h
        msg=ps()
        x1=(1.6*cam_h/100)*(2*(int(point[1])+0)-51)
        x2=(1.6*cam_h/100)*(2*(int(point[0])+0)-51)
        msg.x=x1
        msg.y=x2
        pub.publish(msg)
        print(msg)

def waypoints(path):
        global star
        global start
        global end
        global img
        (r,c)=np.shape(path)
        wp=np.array(start.get_pos())
        start.make_wp()
        for i in range (1,r-2):
                if( path[i-1][0]!=path[i+1][0] and path[i-1][1]!=path[i+1][1]):
                    Spot(int(path[i][0]),int(path[i][1]),gap,50).make_wp()
                    wp=np.vstack([wp,path[i]])
        wp=np.vstack((wp,end.get_pos()))
        end.make_wp()
        cv2.imshow("Image",img) 
        end=time.time()
        print(wp)
        publish(wp[1])  
        cv2.waitKey(300)
                  
                  
def reconstruct_path(came_from, current):
        path=np.zeros((1,2))
        while current in came_from:
            path=np.vstack((path,np.array(current.get_pos())))
            current = came_from[current]
            current.make_path()
        path=np.delete(path,0,0)
        path=np.flipud(path)
        waypoints(path)


                
def algorithm(grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start))
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos())

	open_set_hash = {start}

	while not open_set.empty():

		current = open_set.get()[2]
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		if current != start:
			current.make_closed()

	return False


def make_grid(rows, width):
        global gap
        grid = []
        gap = width // rows
        for i in range(rows):
            grid.append([])
            for j in range(rows):
                spot = Spot(i, j, gap, rows)
                grid[i].append(spot)
        return grid

def main(img, width):
        global ci,ri,cg,rg
        global start
        global end
        ROWS = 50
        grid = make_grid(ROWS, width)    
        end = None
        run = True
        for row in grid:
            for spot in row:
                spot.draw()

        while run and not rospy.is_shutdown():
                spot = grid[ri][ci]
                start=spot
                spot.make_start()
                spot=grid[rg][cg]
                end=spot
                spot.make_end()
                for row in grid:
                    for spot in row:
                        spot.update_neighbors(grid)
                run=algorithm(grid, start, end)

if __name__=='__main__':
    rospy.init_node("Path_planner")
    subg=rospy.Subscriber("/tbot/base_controller/odom",Odometry,callback=pose_callbackg)
    subi=rospy.Subscriber("/pbot/base_controller/odom",Odometry,callback=pose_callbacki)
    # rospy.Subscriber("/bot_pose",pos,callback=pose)
    main(img, width)