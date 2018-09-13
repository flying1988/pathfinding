
# coding: utf-8
# In[ ]:

import heapq
import pickle
import cv2
import os
import datetime

H_ALPHA = 1 # 调h函数的系数，值越大运算速度越快，但是路径越长


def generate_map(map_data_save_name):
    
    #结点，坐标为实际地理坐标，单位为mm，坐标原点为啪啪网咖（二店）一楼平面图的最左上角建筑内沿
    NODES_MM = [(9580, 0), (9580, 5140), (9580, 7810), (9580, 11370),
                (15300, 0), (15300, 5140),
                (18880, 0), (18880, 5140),
                (19070,5140), (19070, 7810), (19070, 11370), (19070, 14230),
                (22460, 0), (22460, 5140),
                (23300, 14230), (23300, 19550),
                (25260, 5140), (25260, 11540), (25260, 14230),
                (26060, 0), (26060, 5140),
                (27000, 11540), (27000, 14230), (27000, 19550),
                (28410, 0), (28410, 5140), (28410, 5400),
                (30620, 14230), (30620, 19020),
                (31200, 14230), (31200, 11540),
                (32080, 0), (32080, 5400), 
                (37140, 11540)]
    
    path_data = []  #存放路径，路径包含有：ID号、起止结点、宽度和曲率
    
    path_data.append({'ID': 0, 'NODES': [(9580, 0), (9580, 5140)], 'WIDTH': 1300, 'CURVATURE': 0})
    path_data.append({'ID': 1, 'NODES': [(9580, 5140), (9580, 7810)], 'WIDTH': 1300, 'CURVATURE': 0})
    path_data.append({'ID': 2, 'NODES': [(9580, 7810), (9580, 11370)], 'WIDTH': 1300, 'CURVATURE': 0})                      

    path_data.append({'ID': 3, 'NODES': [(15300, 0), (15300, 5140)], 'WIDTH': 684, 'CURVATURE': 0})  
    
    path_data.append({'ID': 4, 'NODES': [(18880, 0), (18880, 5140)], 'WIDTH': 706, 'CURVATURE': 0})
    
    path_data.append({'ID': 5, 'NODES': [(19070, 5140), (19070, 7810)], 'WIDTH': 1570, 'CURVATURE': 0})
    path_data.append({'ID': 6, 'NODES': [(19070, 7810), (19070, 11370)], 'WIDTH': 1570, 'CURVATURE': 0})
    path_data.append({'ID': 7, 'NODES': [(19070, 11370), (19070, 14230)], 'WIDTH': 1570, 'CURVATURE': 0})   
    
    path_data.append({'ID': 8, 'NODES': [(22460, 0), (22460, 5140)], 'WIDTH': 706, 'CURVATURE': 0}) 
    
    path_data.append({'ID': 9, 'NODES': [(23300, 14230), (23300, 19550)], 'WIDTH': 700, 'CURVATURE': 0}) 
    
    path_data.append({'ID': 10, 'NODES': [(25260, 5140), (25260, 11540)], 'WIDTH': 1431, 'CURVATURE': 0})                                          
    path_data.append({'ID': 11, 'NODES': [(25260, 11540), (25260, 14230)], 'WIDTH': 1431, 'CURVATURE': 0})
                                          
    
    path_data.append({'ID': 12, 'NODES': [(26060, 0), (26060, 5140)], 'WIDTH': 706, 'CURVATURE': 0}) 
                                         
    
    path_data.append({'ID': 13, 'NODES': [(27000, 11540), (27000, 14230)], 'WIDTH': 706, 'CURVATURE': 0})
    path_data.append({'ID': 14, 'NODES': [(27000, 14230), (27000, 19550)], 'WIDTH': 706, 'CURVATURE': 0})
                                         
                                         
    path_data.append({'ID': 15, 'NODES': [(28410, 5140), (28410, 5400)], 'WIDTH': 897, 'CURVATURE': 0}) 
    path_data.append({'ID': 16, 'NODES': [(28410, 0), (28410, 5140)], 'WIDTH': 897, 'CURVATURE': 0}) 
                                         
    path_data.append({'ID': 17, 'NODES': [(30620, 14230), (30620, 19020)], 'WIDTH': 706, 'CURVATURE': 0}) 
                                         
    path_data.append({'ID': 18, 'NODES': [(31200, 14230), (31200, 11540)], 'WIDTH': 1830, 'CURVATURE': 0}) 
                                         
    path_data.append({'ID': 19, 'NODES': [(32080, 0), (32080, 5400)], 'WIDTH': 651, 'CURVATURE': 0}) 
                                         
    path_data.append({'ID': 20, 'NODES': [(9580, 5140), (15300, 5140)], 'WIDTH': 1250, 'CURVATURE': 0}) 
    path_data.append({'ID': 21, 'NODES': [(15300, 5140), (18880, 5140)], 'WIDTH': 1250, 'CURVATURE': 0}) 
    path_data.append({'ID': 22, 'NODES': [(18880, 5140), (19070, 5140)], 'WIDTH': 1250, 'CURVATURE': 0}) 
    path_data.append({'ID': 23, 'NODES': [(19070, 5140), (22460, 5140)], 'WIDTH': 1250, 'CURVATURE': 0})                                      
    path_data.append({'ID': 24, 'NODES': [(22460, 5140), (25260, 5140)], 'WIDTH': 1250, 'CURVATURE': 0})                                      
    path_data.append({'ID': 25, 'NODES': [(25260, 5140), (26060, 5140)], 'WIDTH': 1250, 'CURVATURE': 0})
    path_data.append({'ID': 26, 'NODES': [(26060, 5140), (28410, 5140)], 'WIDTH': 1250, 'CURVATURE': 0})                           
                                                                                                                                                           
    path_data.append({'ID': 27, 'NODES': [(9580, 7810), (19070, 7810)], 'WIDTH': 706, 'CURVATURE': 0}) 
                                         
    path_data.append({'ID': 28, 'NODES': [(9580, 11370), (19070, 11370)], 'WIDTH': 706, 'CURVATURE': 0}) 
                                          
    path_data.append({'ID': 29, 'NODES': [(25260, 11540), (27000, 11540)], 'WIDTH': 2018, 'CURVATURE': 0})                                     
    path_data.append({'ID': 30, 'NODES': [(27000, 11540), (31200, 11540)], 'WIDTH': 2018, 'CURVATURE': 0})  
    path_data.append({'ID': 31, 'NODES': [(31200, 11540), (37140, 11540)], 'WIDTH': 2018, 'CURVATURE': 0})
                                         
    path_data.append({'ID': 32, 'NODES': [(19070, 14230), (23300, 14230)], 'WIDTH': 1432, 'CURVATURE': 0})
    path_data.append({'ID': 33, 'NODES': [(23300, 14230), (25260, 14230)], 'WIDTH': 1432, 'CURVATURE': 0})                        
    path_data.append({'ID': 34, 'NODES': [(25260, 14230), (27000, 14230)], 'WIDTH': 1432, 'CURVATURE': 0})                                     
    path_data.append({'ID': 35, 'NODES': [(27000, 14230), (30620, 14230)], 'WIDTH': 1432, 'CURVATURE': 0})                                     
    path_data.append({'ID': 36, 'NODES': [(30620, 14230), (31200, 14230)], 'WIDTH': 1432, 'CURVATURE': 0}) 
    
    mf = open(map_data_save_name, 'wb')
    pickle.dump(path_data, mf)
    mf.close()


class My_Map():
    def __init__(self, map_data):
        self.map_data = map_data
        
    def cost(self, from_node, to_node):
        (from_x, from_y) = from_node
        (to_x, to_y) = to_node
        distance = abs(from_x - to_x) + abs(from_y - to_y)
        return distance
    
    def neighbors(self):
        '''
        寻找每个结点的邻结点
        args:
        map_data: 地图数据
    
        returns:
        neighbors_dict: 字典，用于存放各结点的邻结点。该字典的键位结点坐标，值为结点对应的邻结点坐标
        '''
    
        neighbors_dict = {}   #key: node_name; value: the node's neighbors
    
        #初始化结点的邻结点： 
        #寻找到地图中存在的所有结点，将其存放在字典中，结点的地理坐标为字典的键，该结点对应的邻结点为其值，邻结点初始化为空列表
        for path in self.map_data:
            for node in path['NODES']:
                if node not in neighbors_dict.keys():
                    neighbors_dict[node] = self.find_neighbors(node, self.map_data)
    
        return neighbors_dict
    
    def find_neighbors(self, node, map_data):
        '''
        #遍历地图中的每条路径，查看该结点是否在路径中，若在路径中，则找到邻结点，并记录下来
        
        args:
        node: 目标结点
        map_data: 地图信息
        
        returns:
        neighbors_of_node: 列表，用于存放目标结点的邻结点
        '''
        
        neighbors_of_node = []
        for path in map_data:
            if node in path['NODES']:
                if node == path['NODES'][0]:
                    neighbors_of_node.append(path['NODES'][1])
                else:
                    neighbors_of_node.append(path['NODES'][0])
                        
        return neighbors_of_node
    
        
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]
    
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return (abs(x1 - x2) + abs(y1 - y2)) * H_ALPHA
    

def a_star_search(graph, start, goal):
    '''
    A*路径搜索
    args:
    graph:地图数据
    start:起始结点
    goal:目标结点
    
    returns:
    came_from:搜索到路径结点
    cost_so_far:该路径的总代价
    '''
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        
#         (current_x, current_y) = current
#         (goal_x, goal_y) = goal
#         if (abs(current_x-goal_x) + abs(current_y-goal_y)) <= SPEED:
#             came_from[goal] = current
#             break

        for next in graph.neighbors()[current]:
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
        #print(current)
 
    return came_from, cost_so_far

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        #print(path)
        current = came_from[current]
    path.append(start)
    path.reverse()
    print('path: ' + str(len(path)))
    return path

def coordinate_transfer(path_mm):
    '''
    将路径中结点的地理坐标转换成像素坐标，用于在像素地图图片绘制路径。
    需要进行以下步骤：
    1、将实际长度和像素比例进行缩放
    2、坐标原点并不在图片的（0，0），需将像素坐标原点移动到图片的实际位置
    3、地理坐标是（横坐标，纵坐标），像素坐标则是（纵坐标，横坐标），所以需要将两个坐标轴对调
    
    args:
    path_mm:地理坐标路径结点
    
    returns:
    path_pix:转换成像素坐标后的路径结点
    '''
    
    MAP_HEIGHT_MM = 40400    #地图的原始实际宽度，40400mm
    MAP_HEIGHT_PIX = 11200   #地图图像像素的宽度， 11200pix
    (ORIGIN_POSITION_X, ORIGIN_POSITION_Y) = (155, 485)  #定义坐标原点，坐标系原点在图片上位置像素为(155, 465)
    
    mm_per_pix = MAP_HEIGHT_MM / MAP_HEIGHT_PIX  #计算一个像素实际是多少毫米
    
    path_pix = []     #用于保存像素节点的坐标
    for (x, y) in path_mm:
        path_pix.append((((x // mm_per_pix) + ORIGIN_POSITION_X), ((y // mm_per_pix) + ORIGIN_POSITION_Y))) 
    return path_pix

    

def draw_path(map_file, path, result_file):
    '''
    在像素地图上画出规划好的路径
    
    args:
    map_file:无路径的像素地图文件
    path:路径的结点，结点坐标为像素坐标
    result_file:画有路径的像素地图文件
    '''
    #print(path)
    
    line_color = (255, 255, 255)
    line_width = 15
    circle_color = (0, 0, 255)
    circle_radius = 20
    
    my_map = cv2.imread(map_file)
    #绘制路径
    for i in range(len(path)-1):
        #print((int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])))
        cv2.line(my_map, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), line_color, line_width)
    
    #标记关键结点（转弯处结点）
    for i in range(len(path)):
        if i+2 >= len(path):
            break
        #判断path[i]、path[i+1]和path[i+2]三个点是否在同一条直线上
        #首先考虑横坐标相等的情况
        if path[i][0] == path[i+1][0] or path[i+1][0] == path[i+2][0]:
            if path[i][0] != path[i+2][0]:
                cv2.circle(my_map, (int(path[i+1][0]), int(path[i+1][1])), circle_radius, circle_color, -1)
        #其次考虑斜率是否相等
        elif ((path[i][1]-path[i+1][1])/(path[i][0]-path[i+1][0])) != ((path[i+1][1]-path[i+2][1])/(path[i+1][0]-path[i+2][0])):
            cv2.circle(my_map, (int(path[i+1][0]), int(path[i+1][1])), circle_radius, circle_color, -1)
            
    cv2.imwrite(result_file, my_map)
                          
    

map_data_save_path = 'data' #地图存放的路径                 
map_data_save_name = 'map_data.m'  # 地图的文件名
start_position = (9580, 0)      # 起始坐标
goal_position = (31200, 14230)   #目标(9580, 0)坐标

image_orig = 'data/MAP_ORIG.png' #待绘制路径的图片
path_image = 'data/path_image.png'  # 绘有路径的图片 

time_start = datetime.datetime.now()
                          

#step1: 在地图未构建时，构建地图
if map_data_save_name not in os.listdir(map_data_save_path):
    generate_map(os.path.join(map_data_save_path, map_data_save_name))

#step2: 读取已生成的地图
with open(os.path.join(map_data_save_path, map_data_save_name), 'rb') as f:
    map_data = pickle.load(f)
    
#step3:搜索路径
internet_cafe_map = My_Map(map_data)
came_from, cost_so_far = a_star_search(internet_cafe_map, start_position, goal_position)                          
path_mm = reconstruct_path(came_from, start=start_position, goal=goal_position)
                          

#step4: 坐标变换
path_pix = coordinate_transfer(path_mm)
                          
#step5：绘制路径
draw_path(image_orig, path_pix, path_image)
                          
# print(path_mm)
# print(path_pix)
time_end = datetime.datetime.now()
time_cost = (time_end - time_start)
print('time cost: ' + str(time_cost) + 's')

