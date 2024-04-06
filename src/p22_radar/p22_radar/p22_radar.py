import rclpy
from rclpy.node import Node
from radar_msgs.msg import RadarTracks
from radar_msgs.msg import RadarTrack
import serial
import re
import binascii
import array as arr
import serial.tools.list_ports
predata = ""
x_coord = arr.array('f', [0, 0, 0, 0, 0])
y_coord = arr.array('f', [0, 0, 0, 0, 0])
period_ms = 200
port_name = '/dev/ttyUSB0'  

def find_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def is_serial_port_available(port_name):
    return port_name in find_serial_ports()

def parse_one_frame(data):
    #解析一帧完整数据
    global data_length
            
    # 帧头
    head_frame = data[:6]
    # print(head_frame)

    # 数据长度(命令字+命令值)
    data_length = data[6:10]
    print(data_length)

    # 命令字
    order_frame = data[10:12]
    # print(order_frame)

    # 命令值
    data_frame = data[12:12+2*(int(data_length,16)-1)]
    print(data_frame)

    # 校验和
    sum_frame = data[12+2*(int(data_length,16)-1):12+2*(int(data_length,16)-1)+2]
    # print(sum_frame)

    # 帧尾
    tail_frame = data[-6:]
    # print(tail_frame)

    # 帧头判断
    if(head_frame != "ffeedd"):
        print("Wrong starter")

    # 校验和判断
    sum = int(order_frame,16) 
    for i in range(0,(int(data_length,16)-1),1):

        sum += int(data_frame[2*i:2*i+2],16)

    sum = '{:x}'.format(sum)
    # print(sum[-2:])
    if (int(sum[-2:],16) != int(sum_frame,16)):
        print("Wrong sum")


    
    # 帧尾判断
    if(tail_frame != "ddeeff"):
        print("Wrong ender")
      

    # 坐标解析输出 先x后y 单位m


    data_list = re.findall('.{2}',data_frame)

    for i in range(0,int(data_length,16)-1,2):
        x = int(data_list[i],16)
        if (x > 127):
            x = x - 256
        x /= 10

        y = int(data_list[i+1],16)
        if (y > 127):
            y = y - 256
        y /= 10

        x_coord[int(i/2)] = x
        y_coord[int(i/2)] = y
        print((x,y))
class RadarNode(Node):

    def __init__(self):
        super().__init__('p22_radar')
        self.publisher_ = self.create_publisher(RadarTracks, 'radar/detected_objects', (int)(1000/period_ms))
        self.timer_ = self.create_timer(period_ms/1000, self.serial_callback)
        self.serial_port_ = serial.Serial(port = port_name,
                        baudrate = 9600,
                        stopbits = serial.STOPBITS_ONE,
                        parity = serial.PARITY_NONE,
                        rtscts = False,
                        timeout = None,
                        write_timeout = None)

    def serial_callback(self):
        global predata, x_coord, y_coord

        if self.serial_port_.in_waiting > 0:
            radar_tracks_msg = RadarTracks()
            msg1 = RadarTrack()
            msg2 = RadarTrack()
            msg3 = RadarTrack()
            msg4 = RadarTrack()
            msg5 = RadarTrack()
            predata = self.serial_port_.read(self.serial_port_.in_waiting)
            data = predata.hex()
            print(data)
            parse_one_frame(data)

            if(int(data_length,16) >= 3):
                msg1.position.x = x_coord[0]
                msg1.position.y = y_coord[0]
            else:
                msg1.position.x = 0.0
                msg1.position.x = 0.0

            if(int(data_length,16) >= 5):
                msg2.position.x = x_coord[1]
                msg2.position.y = y_coord[1]
            else:
                msg2.position.x = 0.0
                msg2.position.x = 0.0

            if(int(data_length,16) >= 7):   
                msg3.position.x = x_coord[2]
                msg3.position.y = y_coord[2]
            else:
                msg3.position.x = 0.0
                msg3.position.x = 0.0

            if(int(data_length,16) >= 9):  
                msg4.position.x = x_coord[3]
                msg4.position.y = y_coord[3]
            else:
                msg4.position.x = 0.0
                msg4.position.x = 0.0

            if(int(data_length,16) >= 11):     
                msg5.position.x = x_coord[4]
                msg5.position.y = y_coord[4]
            else:
                msg5.position.x = 0.0
                msg5.position.x = 0.0
            radar_tracks_msg.header.stamp = self.get_clock().now().to_msg()
            radar_tracks_msg.header.frame_id = "detected_objects"
            radar_tracks_msg.tracks.append(msg1)
            radar_tracks_msg.tracks.append(msg2)
            radar_tracks_msg.tracks.append(msg3)
            radar_tracks_msg.tracks.append(msg4)
            radar_tracks_msg.tracks.append(msg5)
            self.publisher_.publish(radar_tracks_msg)


class EmptyRadarNode(Node):

    def __init__(self):
        super().__init__('p22_radar')
        self.publisher_ = self.create_publisher(RadarTracks, 'radar/detected_objects', (int)(1000/period_ms))
        self.timer_ = self.create_timer(period_ms/1000, self.publish_empty_message)

    def publish_empty_message(self):
        empty_radar_msg = RadarTracks()
        msg = RadarTrack()
        msg.position.x = 0.0
        msg.position.y = 0.0
        empty_radar_msg.tracks.append(msg)
        self.publisher_.publish(empty_radar_msg)


def main(args=None):
    rclpy.init(args=args)
    if is_serial_port_available(port_name):
        node = RadarNode()
    else:
        node = EmptyRadarNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
