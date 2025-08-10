import rclpy
import time
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from decimal import Decimal
import threading
import matplotlib.pyplot as plt

from bin_packing import Packer, Bin, Item



'''
            depth(z)
              |
              | 
              |
              |
              |
              |
            0 |________________ width(x)
             /
            / 
           /  
          /   
         /    
      height(y)
      
    depth는 확실한데, width랑 height는 반대일 수도 있으니 확인 필요
    구석부터 쌓기 위해서 (0,0,0)이 적재 공간 내 왼쪽 아래 구석으로 설정
    
'''

class PackingNode(Node):
    def __init__(self):
        super().__init__('packing_node')
        self.declare_parameter('destination', 1)
        self.box_separation = self.get_parameter('destination').value
        
        self.publisher = self.create_publisher(Point, 'packing_position', 1)
        self.viz_box_publisher = self.create_publisher(Marker, 'box_marker', 1)
        self.viz_edge_publisher = self.create_publisher(Marker, 'edge_marker', 1)
        self.subscriber = self.create_subscription(PointStamped, 'set_packing_item', self.target_callback, 1)
        self.item_count = 0
        
        self.max_whd = (750, 500, 430)
        
        self.packer_list = []
        self.packer_pos_offset_list = []
        self.box_list = []
        self.color_list=[
            ColorRGBA(r=1.00, g=0.75, b=0.71, a=0.8),
            ColorRGBA(r=0.73, g=0.87, b=0.98, a=0.8),
            ColorRGBA(r=0.78, g=0.90, b=0.79, a=0.8),
            ColorRGBA(r=0.94, g=0.96, b=0.76, a=0.8),
        ]
        
        self.initiallize_packers(self.box_separation)
        
        self.vis_box_marker_id = 0
        self.vis_edge_marker_id = 0
        
    def target_callback(self, data: PointStamped):
            
      whd = data.point
      id = int(data.header.frame_id)
      if id > len(self.packer_list)-1: return
      
      self.packer_list[id].addItem(Item(
        partno='Item{}'.format(str(self.item_count+1)),
        name='Item', 
        typeof='cube',
        WHD=(whd.x, whd.y, whd.z), 
        weight=1,
        level=1,
        loadbear=100,
        updown=False,
        color='#FF0000')
      )
      
      self.packer_list[id].pack(
        bigger_first=False,
        distribute_items=False,
        fix_point=True,
        check_stable=True,
        support_surface_ratio=0.75,
        number_of_decimals=0
      )
      
      if len(self.packer_list[id].bins[0].unfitted_items) > 0: 
        self.get_logger().warn(f'No Room For Packing. Packing Destination ID: {id}')
        self.packer_list[id].items=[]
        self.packer_list[id].unfitted_items=[]
        return
      
      volume = self.packer_list[id].bins[0].width * self.packer_list[id].bins[0].height * self.packer_list[id].bins[0].depth
      volume_t = 0
      for item in self.box_list[id].items:
          volume_t += float(item.width) * float(item.height) * float(item.depth)
      # print(len(self.box.items))
      print("position : ",self.box_list[id].items[-1].position)
      print('space utilization : {}%'.format(round(volume_t / float(volume) * 100 ,2)))
      print('----------------------------------------')
      
      
      msg = Point()
      last_item = self.box_list[id].items[-1]
      last_item_pos = last_item.position
      if last_item.rotation_type == 0:
        msg.x = float(last_item_pos[0]) + float(last_item.width) * 0.5 + self.packer_pos_offset_list[id][0]
        msg.y = float(last_item_pos[1]) + float(last_item.height) * 0.5 + self.packer_pos_offset_list[id][0]
        msg.z = float(last_item_pos[2]) + float(last_item.depth) * 0.5 + self.packer_pos_offset_list[id][0]
      else:
        msg.x = float(last_item_pos[0]) + float(last_item.height) * 0.5 + self.packer_pos_offset_list[id][0]
        msg.y = float(last_item_pos[1]) + float(last_item.width) * 0.5 + self.packer_pos_offset_list[id][0]
        msg.z = float(last_item_pos[2]) + float(last_item.depth) * 0.5 + self.packer_pos_offset_list[id][0]
      
      self.publisher.publish(msg)
      self.get_logger().info(f'Publishing Point: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
      self.visualize_boxes(id,
                          last_item, 
                          line_thickness=0.05,
                          box_color=self.color_list[id], 
                          edge_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0))
      
      self.item_count += 1
      self.packer_list[id].items=[]
    
    
    def initiallize_packers(self, box_seperation: int):
        packer_list = []
        packer_pos_list = []
        box_list = []
        
        for i in range(box_seperation):
            packer_list.append(Packer())
        
        self.packer_list = packer_list  
        packers_count = len(self.packer_list)
        
        if packers_count == 1:
            box = Bin(
              partno='Box',
              WHD=self.max_whd,
              max_weight=10000,
              corner=0,
              put_type=0
            )
            self.allocate_bin_size(self.packer_list[0], box)
            packer_pos_list = [[0,0,0]]
            box_list.append(box)
            self.packer_pos_offset_list = packer_pos_list
            self.box_list = box_list
            
        elif packers_count == 2:
            whd = (self.max_whd[0]*0.5, self.max_whd[1], self.max_whd[2])
            for i in range(packers_count):
                box = Bin(
                  partno='Box',
                  WHD=whd,
                  max_weight=10000,
                  corner=0,
                  put_type=0
                )
                self.allocate_bin_size(self.packer_list[i], box)
                box_list.append(box)
            packer_pos_list = [
                                [0,0,0],
                                [whd[0],0,0]
                              ]
            self.packer_pos_offset_list = packer_pos_list
            self.box_list = box_list
            
        elif packers_count == 3:
            whd = (self.max_whd[0]*0.333, self.max_whd[1], self.max_whd[2])
            for i in range(packers_count):
                box = Bin(
                  partno='Box',
                  WHD=whd,
                  max_weight=10000,
                  corner=0,
                  put_type=0
                )
                self.allocate_bin_size(self.packer_list[i], box)
                box_list.append(box)
            packer_pos_list = [
                                [0,0,0],
                                [whd[0],0,0],
                                [whd[0]*2,0,0]
                              ]
            self.packer_pos_offset_list = packer_pos_list
            self.box_list = box_list
            
        elif packers_count == 4:
            whd = (self.max_whd[0]*0.5, self.max_whd[1] *0.5, self.max_whd[2])
            for i in range(packers_count):
                box = Bin(
                  partno='Box',
                  WHD=whd,
                  max_weight=10000,
                  corner=0,
                  put_type=0
                )
                self.allocate_bin_size(self.packer_list[i], box)
                box_list.append(box)
            packer_pos_list = [
                                [0,0,0],
                                [0,whd[1],0],
                                [whd[0],0,0],
                                [whd[0],whd[1],0],
                              ]
            self.packer_pos_offset_list = packer_pos_list
            self.box_list = box_list
            
        else:
            self.get_logger().error(f'Too many separation! Set default separation: 1')
            self.initiallize_packers(1)

              
      
    def allocate_bin_size(self, packer: Packer, box: Bin):
      packer.addBin(box)
      
      
    def visualize_boxes(self, id, item, line_thickness, box_color, edge_color):
      marker = Marker()
      marker.header.frame_id ="map"
      marker.header.stamp = self.get_clock().now().to_msg()
      marker.ns = "Box"
      marker.id = self.vis_box_marker_id
      
      marker.type = Marker.CUBE
      marker.action = Marker.ADD
      if item.rotation_type == 0:
        marker.pose.position.x = (float(item.position[0]) + float(item.width) * 0.5 + self.packer_pos_offset_list[id][0]) * 0.01
        marker.pose.position.y = (float(item.position[1]) + float(item.height) * 0.5 + self.packer_pos_offset_list[id][1]) * 0.01
        marker.pose.position.z = (float(item.position[2]) + float(item.depth) * 0.5 + self.packer_pos_offset_list[id][2]) * 0.01
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
      else:
        marker.pose.position.x = (float(item.position[0]) + float(item.height) * 0.5 + self.packer_pos_offset_list[id][0]) * 0.01
        marker.pose.position.y = (float(item.position[1]) + float(item.width) * 0.5 + self.packer_pos_offset_list[id][1]) * 0.01
        marker.pose.position.z = (float(item.position[2]) + float(item.depth) * 0.5 + self.packer_pos_offset_list[id][2]) * 0.01
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.7071068
        marker.pose.orientation.w = 0.7071068
      
      # cm 단위로 설정
      marker.scale.x = float(item.width) * 0.01
      marker.scale.y = float(item.height) * 0.01
      marker.scale.z = float(item.depth) * 0.01
      
      marker.color = box_color

      marker.lifetime.sec = 0
      self.vis_box_marker_id += 1 

      line_marker = Marker()
      line_marker.header.frame_id = "map"
      line_marker.header.stamp = self.get_clock().now().to_msg()
      line_marker.ns = "BoxEdges"
      line_marker.id = self.vis_edge_marker_id
      
      line_marker.type = Marker.LINE_LIST
      line_marker.action = Marker.ADD

      line_marker.pose = marker.pose
      
      line_marker.scale.x = line_thickness
      
      line_marker.color = edge_color
      line_marker.lifetime.sec = 0
      
      hx = float(item.width) * 0.5 * 0.01
      hy = float(item.height) * 0.5 * 0.01
      hz = float(item.depth) * 0.5 * 0.01
      
      vertices = [
          Point(x=-hx, y=-hy, z=-hz),
          Point(x= hx, y=-hy, z=-hz),
          Point(x= hx, y= hy, z=-hz),
          Point(x=-hx, y= hy, z=-hz),
          Point(x=-hx, y=-hy, z= hz),
          Point(x= hx, y=-hy, z= hz),
          Point(x= hx, y= hy, z= hz),
          Point(x=-hx, y= hy, z= hz) 
      ]
      edges_indices = [
          (0, 1), (1, 2), (2, 3), (3, 0),
          (4, 5), (5, 6), (6, 7), (7, 4),
          (0, 4), (1, 5), (2, 6), (3, 7)
      ]
      
      line_marker.points = []
      for idx1, idx2 in edges_indices:
          line_marker.points.append(vertices[idx1])
          line_marker.points.append(vertices[idx2])
      self.vis_edge_marker_id += 1
          
      self.viz_box_publisher.publish(marker)
      self.viz_edge_publisher.publish(line_marker)
      self.get_logger().info(f'Published a CUBE Marker with ID: {marker.id}, Destination ID: {id} at ({marker.pose.position.x*100}, {marker.pose.position.y*100}, {marker.pose.position.z*100})')
      
      
def main(args=None):
  
    rclpy.init(args=args)
    
    packing_node = PackingNode()
    packing_node.get_logger().info('======================================================')
    packing_node.get_logger().info(f'Packing Destinaion is {packing_node.box_separation}')
    time.sleep(1)
    
    container = Item(partno='Container',
                    name='Container',
                    typeof='cube',
                    WHD=packing_node.max_whd,
                    weight=1,
                    level=1,
                    loadbear=0,
                    updown=False,
                    color='white')
    
    packing_node.visualize_boxes(
                          id=0,
                          item=container, 
                          line_thickness=0.1,
                          box_color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.0), 
                          edge_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
                          )
    
    rclpy.spin(packing_node)

    packing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()