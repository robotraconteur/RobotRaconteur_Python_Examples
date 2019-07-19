# This example will show a PyQt5 window of all discovered devices.
# Devices can then be selected to drive with a gamepad

import time
import pygame
import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from RobotRaconteur.Client import *
import traceback
import threading
from urllib.parse import urlparse

if (sys.version_info > (3, 0)):
    def cmp(x, y):
        return (x > y) - (x < y)
    
   
class RobotClient(QObject):
    
    detected_nodes_updated = pyqtSignal()
    drive_error = pyqtSignal()
    
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
        self.robot_info_widget = None
        self.update_lock = threading.Lock()
    
        self.detected_nodes_updated.connect(self.update_subscriber_window)
        self.drive_keep_going = False
    
    def run(self):
        while True:
            success, robot_url, webcam_url = self.subscriber_window()
            if not success:
                return       
            self.drive_window(robot_url, webcam_url)
    
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
            
            #Find services, assume that nodes with the same IP address are related
            for s in self.service_subscriber.GetDetectedServiceInfo2().values():
                if s.RootObjectType == 'experimental.create2.Create':
                    webcam_info=None
                    for w in self.service_subscriber.GetDetectedServiceInfo2().values():
                        if (w.RootObjectType == 'experimental.createwebcam2.WebcamHost' \
                          and urlparse(s.ConnectionURL[0]).hostname == urlparse(w.ConnectionURL[0]).hostname):
                            webcam_info=w                    
                    list_values.append(RobotQListWidgetItem(s,webcam_info))
                        
            current_item = l.currentItem()
            if (current_item is not None):
                current_nodeid = current_item.robot_service_info.NodeID
                current_service_name = current_item.robot_service_info.Name
            else:
                current_nodeid = None
                current_service_name= None
                             
            l.clear()
            
            if len(list_values) == 0:
                if self.robot_info_widget is not None:
                    self.robot_info_widget.setText("")
                return
                        
            for lv in list_values:
                l.addItem(lv)
            for lv in list_values:
                if current_nodeid == lv.robot_service_info.NodeID \
                  and current_service_name == lv.robot_service_info.Name:
                    l.setCurrentItem(lv)
            
    
    def subscriber_window(self):
    
        w = QFrame()
        w.resize(850,500)
                
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
                            
        robot_selected=False
        robot_service_info=None
        webcam_service_info=None                    
        
        def select_button_pressed():
            current_item = robot_list_widget.currentItem()
            if current_item is None:
                return 
            item_selected(current_item)
            
        def item_selected(current_item):
            
            nonlocal robot_selected
            nonlocal robot_service_info
            nonlocal webcam_service_info
            
            robot_selected=True
            robot_service_info=current_item.robot_service_info
            webcam_service_info=current_item.webcam_service_info
            
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
                        "    URL:      " + robot_service_info.ConnectionURL[0] + "\n\n"
                    
                    webcam_service_info=current_item.webcam_service_info
                    if webcam_service_info is None:
                        info_text += "Webcam not found!"
                    else:
                        info_text += "Webcam:\n" \
                        "    NodeID:   " + str(webcam_service_info.NodeID) + "\n" \
                        "    NodeName: " + webcam_service_info.NodeName + "\n" \
                        "    Service:  "  + webcam_service_info.Name + "\n" \
                        "    Type:     " + webcam_service_info.RootObjectType + "\n" \
                        "    URL:      " + webcam_service_info.ConnectionURL[0] + "\n\n"
                        
                    robot_info.setText(info_text)
            except:
                traceback.print_exc()
        
        select_button_widget.clicked.connect(select_button_pressed)
        robot_list_widget.itemDoubleClicked.connect(item_selected)
        robot_list_widget.itemSelectionChanged.connect(selection_changed)
        
        self.robot_list_widget=robot_list_widget
        self.robot_info_widget=robot_info
        try:
            self.update_subscriber_window()
            w.show()
            self.app.exec_()
        finally:        
            self.robot_list_widget = None
            self.robot_info_widget = None 
                
        return robot_selected,robot_service_info,webcam_service_info
    
    def drive_window(self, robot_service_info, webcam_service_info):
    
    
        frame_lock = threading.Lock()
        current_frame = None
        current_frame_time = 0
        
        def new_frame(pipe_ep):
            nonlocal current_frame
            nonlocal current_frame_time
            t=time.time()
            with frame_lock:
                while (pipe_ep.Available > 0):       
                    current_frame=pipe_ep.ReceivePacket()
                    current_frame_time = t
        try:
            robot = RRN.ConnectService(robot_service_info.ConnectionURL)
        except Exception as e:            
            traceback.print_exc()
            QMessageBox.critical(None,"Robot Raconteur Connection Error","Could not connect to robot: " + str(e))            
            return     
    
        webcam=None
    
        if (webcam_service_info is not None):
            try:
                webcam_host = RRN.ConnectService(webcam_service_info.ConnectionURL[0])
                webcam=webcam_host.get_Webcams(0)
                p=webcam.FrameStream.Connect(-1)
                
                p.PacketReceivedEvent+=new_frame
                try:
                    webcam.StartStreaming()
                except: pass
            except Exception as e:            
                traceback.print_exc()
                RRN.DisconnectService(robot)
                QMessageBox.critical(None,"Robot Raconteur Connection Error","Could not connect to webcam: " + str(e))                
                return
                
        try:
            w = QWidget()
            w.resize(850,500)
            
            image_label = QLabel()
            close_button_widget = QPushButton("Close")
                    
            close_button_widget.clicked.connect(lambda: w.close())
                    
            vbox = QVBoxLayout()
            vbox.addWidget(image_label)
            vbox.addWidget(close_button_widget)            
            w.setLayout(vbox)
            
            image_label.setFixedHeight(480)
            image_label_font = image_label.font()
            image_label_font.setPixelSize(40)
            image_label.setFont(image_label_font)
            
            def image_timer_cb():                
                try:   
                    with frame_lock:
                        if (time.time() - current_frame_time < 5):
                            img=current_frame
                        else:
                            img=None                
                    
                    if img is not None:                    
                        pixmap1 = QPixmap(QImage(img.data, img.width, img.height, img.step, QImage.Format_RGB888))
                        pixmap2 = pixmap1.scaledToHeight(480)
                        
                        image_label.setPixmap(pixmap2)        
                        image_label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
                        
                    else:
                        image_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                        image_label.clear()
                        image_label.setText("No Image")
                except:
                    traceback.print_exc()
            
            timer = QTimer(w)
            timer.timeout.connect(image_timer_cb)
            timer.setInterval(100)
            timer.start()
                                    
            w.show()
            
            def drive_error_cb():
                QMessageBox.critical(None,"Robot Raconteur Connection Error","Robot drive command failed")
                w.close()
            
            self.drive_error.connect(drive_error_cb)            
            
            self.drive_keep_going = True
            drive_thread=threading.Thread(target=lambda: self.drive(robot,webcam))
            drive_thread.start()
            
            self.app.exec_()
            timer.stop()
        finally:
            self.drive_keep_going = False
            time.sleep(0.5)
            try:
                RRN.DisconnectService(robot)
            except: pass
            if webcam is not None:
                try:
                    webcam.StopStreaming()
                except:
                    pass
                try:
                    RRN.DisconnectService(webcam)
                except: pass
        
    
    def drive(self,robot,webcam):
                        
        try:      
            
            while self.drive_keep_going:
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
                       
                #Delay for 20 ms
                self.clock.tick(20)        
                
        except:
            traceback.print_exc()
            self.drive_error.emit()            
            
class RobotQListWidgetItem(QListWidgetItem):
    def __init__(self, robot_service_info, webcam_service_info):
        super(RobotQListWidgetItem,self).__init__()
        self.robot_service_info = robot_service_info
        self.webcam_service_info = webcam_service_info
        self.setText(robot_service_info.NodeName)

def main():
    
    app=QApplication(sys.argv)        
    icon = QIcon('RRIcon.bmp')
    app.setWindowIcon(icon)    
    with RR.ClientNodeSetup():
        c = RobotClient(app)
        c.run()
      
if __name__ == '__main__':
    main()
