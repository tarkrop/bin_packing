import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from decimal import Decimal

from bin_packing import Packer, Bin, Item, Painter



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
        self.publisher = self.create_publisher(Point, 'packing_position', 1)
        self.subscriber = self.create_subscription(Point, 'set_packing_item', self.target_callback, 1)
        
        self.item_count = 0
        self.fail = False
        
        self.packer = Packer()
        self.box = Bin(
            partno='Box',
            WHD=(75,50,43), # 적재 공간의 크기: ( Width, Height, Depth ) -> ( x, y, z)
            max_weight=10000,
            corner=0,
            put_type=0
        )
        self.packer.addBin(self.box)
        
    def target_callback(self, data: Point):
      
      if self.fail: return 
      
      whd = data
      
      self.packer.addItem(Item(
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
      
      self.packer.pack(
        bigger_first=False,
        distribute_items=False,
        fix_point=True,
        check_stable=True,
        support_surface_ratio=0.75,
        number_of_decimals=0
      )
      
      if len(self.packer.bins[0].unfitted_items) > 0: 
        self.get_logger().info(f'Packing Failed')
        self.fail = True
        return
      
      volume = self.packer.bins[0].width * self.packer.bins[0].height * self.packer.bins[0].depth
      volume_t = 0
      for item in self.box.items:
          volume_t += float(item.width) * float(item.height) * float(item.depth)
      print(len(self.box.items))
      print("position : ",self.box.items[-1].position)
      print('space utilization : {}%'.format(round(volume_t / float(volume) * 100 ,2)))
      print('----------------------------------------')
      
      
      msg = Point()
      last_item = self.box.items[-1]
      last_item_pos = last_item.position
      msg.x = float(last_item_pos[0]) + float(last_item.width) * 0.5
      msg.y = float(last_item_pos[1]) + float(last_item.height) * 0.5
      msg.z = float(last_item_pos[2]) + float(last_item.depth) * 0.5
      
      self.publisher.publish(msg)
      self.get_logger().info(f'Publishing Point: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
      
      self.item_count += 1
      self.packer.items=[]


def main(args=None):
    rclpy.init(args=args)

    packing_node = PackingNode()

    rclpy.spin(packing_node)

    packing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()