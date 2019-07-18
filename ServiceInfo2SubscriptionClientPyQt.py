# This example will show a PyQT window of all discovered devices.
# Devices can then be selected to drive with a gamepad

import time
import numpy as np
import pygame
import sys
import cv2
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from RobotRaconteur.Client import *
import traceback
import threading

if (sys.version_info > (3, 0)):
    def cmp(x, y):
        return (x > y) - (x < y)
    
   
class RobotClient(QObject):
    
    detected_nodes_updated = pyqtSignal()
    
    def __init__(self, app):
        super(RobotClient,self).__init__(app)       
        
        self.app=app
        
        #Init the joystick
        pygame.init()
        pygame.joystick.init()
        self.joy=pygame.joystick.Joystick(0)
        self.joy.init()
        self.clock=pygame.time.Clock()
                               
        self.current_frame=None
                       
        self.service_subscriber = RRN.SubscribeServiceInfo2(['experimental.create2.Create','experimental.createwebcam2.WebcamHost'])
        self.service_subscriber.ServiceDetected += self.service_detected
        self.service_subscriber.ServiceLost += self.service_lost
        
        self.robot_list_widget = None
        self.update_lock = threading.Lock()
    
        self.detected_nodes_updated.connect(self.update_subscriber_window)
    
    def run(self):
        robot_url, webcam_url = self.subscriber_window()
        if (robot_url is None and webcam_url is None):
            return
    
    def service_detected(self, subscription, client_id, client_info):
        self.detected_nodes_updated.emit()
            
    def service_lost(self, subscription, client_id, client_info):
        self.detected_nodes_updated.emit()
                
    def update_subscriber_window(self):
        with self.update_lock:
            l = self.robot_list_widget
            if l is None:
                return
            list_values = []
            
            for s in self.service_subscriber.GetDetectedServiceInfo2().values():
                if s.RootObjectType == 'experimental.create2.Create':
                    list_values.append(RobotQListWidgetItem(s,None))
                        
            current_item = l.currentItem()
            if (current_item is not None):
                current_nodeid = current_item.robot_service_info.NodeID
                current_service_name = current_item.robot_service_info.Name
            else:
                current_nodeid = None
                current_service_name= None
                             
            l.clear()
                        
            for lv in list_values:
                l.addItem(lv)
            for lv in list_values:
                if current_nodeid == lv.robot_service_info.NodeID \
                  and current_service_name == lv.robot_service_info.Name:
                    l.setCurrentItem(lv)
            
    
    def subscriber_window(self):
    
        w = QWidget()
        w.resize(850,500)
        #w.move(300,300)
        
        robot_list_widget = QListWidget()
        select_button_widget = QPushButton("Select Robot")
        robot_info = QLabel()
        robot_info.setFixedHeight(200)
        robot_info_font = robot_info.font()
        robot_info_font.setFamily("Courier New")
        robot_info.setFont(robot_info_font)
        robot_info.setTextInteractionFlags(Qt.TextSelectableByMouse |
                                            Qt.TextSelectableByKeyboard)
        robot_info.setCursor(QCursor(Qt.IBeamCursor))
        robot_info.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        
        vbox = QVBoxLayout()
        vbox.addWidget(robot_list_widget)        
        vbox.addWidget(robot_info)
        vbox.addWidget(select_button_widget)
        w.setLayout(vbox)
        
        w.setWindowTitle("Available robots")
                            
        def select_button_pressed():
            current_item = robot_list_widget.currentItem()
            if current_item is None:
                return 
            item_selected(current_item)
            
        def item_selected(current_item):
            
            robot_service_info=current_item.robot_service_info
            print(robot_service_info.ConnectionURL[0])
            w.close()
            
        
        def selection_changed():
            try:
                current_item = robot_list_widget.currentItem()
                if current_item is None:
                    robot_info.setText("")
                else:
                    robot_service_info=current_item.robot_service_info
                    info_text = "Robot:\n" \
                        "    NodeID:   " + str(robot_service_info.NodeID) + "\n" \
                        "    NodeName: " + robot_service_info.NodeName + "\n" \
                        "    Service:  "  + robot_service_info.Name + "\n" \
                        "    Type:     " + robot_service_info.RootObjectType + "\n" \
                        "    URL:      " + robot_service_info.ConnectionURL[0]
                        
                    robot_info.setText(info_text)
            except:
                traceback.print_exc()
        
        select_button_widget.clicked.connect(select_button_pressed)
        robot_list_widget.itemDoubleClicked.connect(item_selected)
        robot_list_widget.itemSelectionChanged.connect(selection_changed)
        
        self.robot_list_widget=robot_list_widget
        self.update_subscriber_window()
        w.show()
                   
        self.app.exec_()
        self.robot_list_widget = None
                
        return None,None
    
    def drive(self,robot,webcam):
                        
        
        #Initialize the webcam    
        c=webcam.get_Webcams(0)
        p=c.FrameStream.Connect(-1)
        p.PacketReceivedEvent+=self.new_frame
        try:
            c.StartStreaming()
        except: pass
        cv2.namedWindow("Image")
        
        webcam_downsample=0
        
        
        while True:
            #Loop reading the joysticks and adjust to correct drive parameters
            for event in pygame.event.get():
                pass
    
            speed=0
            radius=32767
    
            x=self.joy.get_axis(0)
            if (abs(x)<.2):
                x=0
            else:
                x=(abs(x)-.2)/.8*cmp(x,0)
    
            y=-self.joy.get_axis(1)
            if (abs(y)<.2):
                y=0
            else:
                y=(abs(y)-.2)/.8*cmp(y,0)
    
    
            if (y==0):
                if (x<0 and x!=0):
                    radius=1
                if (x>0 and x!=0):
                    radius=-1
                if (x!=0):
                    speed=int(abs(x)*200.0)
            else:
                speed=int(y*200.0)
                if (x!=0):
                    radius=int(-(1-abs(x))*5000*cmp(x,0))
                    if (radius==0):
                        radius=-cmp(x,0)
    
    
            #Write out the drive command to the robot
            robot.Drive(speed,radius)
    
            if (webcam_downsample > 5):
                webcam_downsample = 0
                if (not self.current_frame is None):
                    cv2.imshow("Image",self.current_frame)
                    if cv2.waitKey(50)!=-1:
                        break
            else:
                webcam_downsample += 1
    
            #Delay for 20 ms
            self.clock.tick(20)
        
        cv2.destroyAllWindows()
        p.Close()
        c.StopStreaming()
            
        RRN.DisconnectService(robot)
        RRN.DisconnectService(webcam)
    
    def new_frame(self,pipe_ep):        
    
        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            #Receive the packet
            image=pipe_ep.ReceivePacket()
            #Convert the packet to an image and set the global variable
            self.current_frame=WebcamImageToMat(image)
        
class RobotQListWidgetItem(QListWidgetItem):
    def __init__(self, robot_service_info, webcam_service_info):
        super(RobotQListWidgetItem,self).__init__()
        self.robot_service_info = robot_service_info
        self.webcam_service_info = webcam_service_info
        self.setText(robot_service_info.NodeName)

#Function to take the data structure returned from the Webcam service
#and convert it to an OpenCV array
def WebcamImageToMat(image):
    frame2=image.data.reshape([image.height, image.width, 3], order='C')
    return frame2

def main():
    
    app=QApplication(sys.argv)
    
    with RR.ClientNodeSetup():
        c = RobotClient(app)
        c.run()
      
if __name__ == '__main__':
    main()
