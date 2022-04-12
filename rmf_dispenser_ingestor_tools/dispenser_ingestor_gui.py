from tkinter import *
import tkinter.font as font
from threading import Thread, Lock
from time import sleep
import sys
import argparse

from matplotlib.pyplot import grid
from numpy import pad

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rmf_dispenser_msgs.msg import DispenserState
from rmf_dispenser_msgs.msg import DispenserRequest
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_ingestor_msgs.msg import IngestorState
from rmf_ingestor_msgs.msg import IngestorRequest
from rmf_ingestor_msgs.msg import IngestorResult
from rmf_task_msgs.msg import TaskType, Delivery, Loop
from rmf_task_msgs.srv import SubmitTask, CancelTask

class DispenserIngestorGui(Node):
    def __init__(self, dispenser_name, dispenser_location, ingestor_name, ingestor_location):
        self.GUI_name = "Dispenser_ingestor_GUI"
        self.dispenser_name = dispenser_name
        self.ingestor_name = ingestor_name
        self.pickup_place = dispenser_location
        self.dropoff_place = ingestor_location
        super().__init__(self.GUI_name)

        #Initialize GUI elements
        self.root = Tk()
        self.root.title("Manual RMF Dispenser Ingestor GUI")
        self.root.geometry("1920x1080")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.f1 = None
        self.f2 = None
        self.f3 = None
        self.f4 = None
        self.f5 = None
        self.f6 = None
        self.f7 = None
        self.task = None

        self.my_font = font.Font(family="Helvetica", size=20, weight="bold")

        self.start_frame()

        # Initialize the dispenser state and list of served requests
        self.dispenser_state_msg = DispenserState()
        self.dispenser_state_msg.guid = self.dispenser_name
        self.dispenser_state_msg.mode = DispenserState().IDLE
        self.dispenser_state_mutex = Lock()
        self.served_dispenser_requests_mutex = Lock()
        self.served_dispenser_requests = []

        # Initialize the ingestor state and list of served requests
        self.ingestor_state_msg = IngestorState()
        self.ingestor_state_msg.guid = self.ingestor_name
        self.ingestor_state_msg.mode = IngestorState().IDLE
        self.ingestor_state_mutex = Lock()
        self.served_ingestor_requests_mutex = Lock()
        self.served_ingestor_requests = []

        # Set up publishers, subscribers, etc
        self.dispenser_state_timer = self.create_timer(2.0, self.dispenser_state_timer_callback)
        self.dispenser_state_pub = self.create_publisher(DispenserState, 'dispenser_states', 1)
        self.dispenser_result_pub = self.create_publisher(DispenserResult, 'dispenser_results', 1)
        self.dispenser_request_sub = self.create_subscription(DispenserRequest, "dispenser_requests", self.dispenser_requests_callback, 1)

        self.ingestor_state_timer = self.create_timer(2.0, self.ingestor_state_timer_callback)
        self.ingestor_state_pub = self.create_publisher(IngestorState, 'ingestor_states', 1)
        self.ingestor_result_pub = self.create_publisher(IngestorResult, 'ingestor_results', 1)
        self.ingestor_request_sub = self.create_subscription(IngestorRequest, "ingestor_requests", self.ingestor_requests_callback, 1)

        self.node = rclpy.create_node('task_requester_from_dispenser_ingestor_gui')
        self.submit_task_srv = self.node.create_client(SubmitTask, '/submit_task')
        self.cancel_task_srv = self.node.create_client(CancelTask, '/cancel_task')
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.node.set_parameters([param])

    def destroy_frames(self):
        for frame in (self.f1, self.f2, self.f3, self.f4, self.f5, self.f6, self.f7):
            if frame != None:
                frame.destroy()
   
    def start_delivery(self):
        req_msg = SubmitTask.Request()
        req_msg.requester = self.GUI_name 
        req_msg.description.priority.value = 0
        req_msg.description.task_type.type = TaskType.TYPE_DELIVERY
        delivery = Delivery()
        delivery.pickup_place_name = self.pickup_place
        delivery.pickup_dispenser = self.dispenser_name
        delivery.dropoff_ingestor = self.ingestor_name
        delivery.dropoff_place_name = self.dropoff_place
        req_msg.description.delivery = delivery
        rmf_start_time = self.node.get_clock().now().to_msg()
        req_msg.description.start_time = rmf_start_time

        rclpy.spin_once(self.node, timeout_sec=1.0)
        future = self.submit_task_srv.call_async(req_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        self.task = future.result()
        self.waiting_frame()
    
    def go_back_to_start(self):
        req_msg = SubmitTask.Request()
        req_msg.requester = self.GUI_name 
        req_msg.description.priority.value = 0
        req_msg.description.task_type.type = TaskType.TYPE_LOOP
        loop = Loop()
        loop.num_loops = 1
        loop.start_name = self.dropoff_place
        loop.finish_name = self.pickup_place
        req_msg.description.loop = loop
        rmf_start_time = self.node.get_clock().now().to_msg()
        req_msg.description.start_time = rmf_start_time
        
        rclpy.spin_once(self.node, timeout_sec=1.0)
        future = self.submit_task_srv.call_async(req_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        self.going_to_start_frame()


    def cancel_delivery(self):
        cancel_req = CancelTask.Request()
        cancel_req.requester = self.GUI_name
        cancel_req.task_id = self.task.task_id
        rclpy.spin_once(self.node, timeout_sec=1.0)
        future = self.cancel_task_srv.call_async(cancel_req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        self.f5.destroy()
        self.start_frame()

    def start_frame(self):
        self.destroy_frames()
        self.f1 = Frame(self.root)
        self.f1.grid(row=0, column=0)
        self.user_button1 = Button(self.f1, text="Begin delivery", command=self.start_delivery, height=10, width=30, font=self.my_font, bg="green3", activebackground='green2')
        self.user_button1.grid(pady=10, padx=20)
    
    def waiting_frame(self):
        self.destroy_frames()
        self.f2 = Frame(self.root)
        self.f2.grid(row=0, column=0)
        self.label2 = Label(self.f2, font=self.my_font, text="Transportation request received. Please wait")
        self.label2.grid(pady=10, padx=20)

    def dispenser_frame(self):
        self.destroy_frames()
        self.f3 = Frame(self.root)
        self.f3.grid(row=0, column=0)
        self.label3 = Label(self.f3, font=self.my_font, text="Is everything loaded on the robot?")
        self.label3.grid(pady=10, padx=20)
        self.user_button3 = Button(self.f3, text="Everything is loaded", height=10, width=30, command=self.user_button_click_dispenser, font=self.my_font, bg="green3", activebackground='green2')
        self.user_button3.grid(pady=(0, 10), padx=20)
    
    def on_delivery_frame(self):
        self.destroy_frames()
        self.f4 = Frame(self.root)
        self.f4.grid(row=0, column=0)
        self.label4 = Label(self.f4, font=self.my_font, text="Delivery in progress")
        self.label4.grid(pady=10, padx=20)
        self.user_button4 = Button(self.f4, text="Cancel delivery", height=10, width=30, command=self.confirm_cancel_frame, font=self.my_font, bg="red2", activebackground='red3')
        self.user_button4.grid(pady=(0, 10), padx=20)

    def confirm_cancel_frame(self):
        self.destroy_frames()
        self.f5 = Frame(self.root)
        self.f5.grid(row=0, column=0)
        self.label5 = Label(self.f5, font=self.my_font, text="Are you sure you want to cancel the delivery?")
        self.user_button_cancel = Button(self.f5, text="Cancel", height=8, width=20, command=self.cancel_delivery, font=self.my_font, bg="red2", activebackground='red3')
        self.user_button_continue = Button(self.f5, text="Continue", height=8, width=20, command=self.on_delivery_frame, font=self.my_font, bg="green3", activebackground='green2')
        self.label5.grid(row=0, column=0, columnspan=2)
        self.user_button_cancel.grid(row=1, column=0)
        self.user_button_continue.grid(row=1, column=1)

    def ingestor_frame(self):
        self.destroy_frames()
        self.f6 = Frame(self.root)
        self.f6.grid(row=0, column=0)
        self.label6 = Label(self.f6, font=self.my_font, text="Is everything unloaded from the robot?")
        self.label6.grid(pady=10, padx=20)
        self.user_button6 = Button(self.f6, text="Everything is unloaded", height=10, width=30, command=self.user_button_click_ingestor, font=self.my_font, bg="green3", activebackground='green2')
        self.user_button6.grid(pady=(0, 10), padx=20)

    def going_to_start_frame(self):
        self.destroy_frames()
        self.f7 = Frame(self.root)
        self.f7.grid(row=0, column=0)
        self.label7 = Label(self.f7, font=self.my_font, text="Going back to the start")
        self.label7.grid(pady=10, padx=20)
        sleep(10)
        self.start_frame()

    def dispenser_state_timer_callback(self):
        self.dispenser_state_mutex.acquire()
        self.dispenser_state_msg.time = self.get_clock().now().to_msg()
        self.get_logger().debug('Publishing: "{0}"'.format(self.dispenser_state_msg))
        self.dispenser_state_pub.publish(self.dispenser_state_msg)
        self.dispenser_state_mutex.release()

    def dispenser_requests_callback(self, msg):
        print("received a dispensing request for '" + msg.request_guid + "'")
        self.dispenser_state_mutex.acquire()

        # Make sure that the dispenser is idling; the request is for this dispenser and 
        # this request has not been served before
        if self.dispenser_state_msg.mode == DispenserState().IDLE \
        and msg.target_guid == self.dispenser_name \
        and self.is_new_dispenser_request(msg.request_guid):
            self.dispenser_state_msg.mode = DispenserState().BUSY
            #self.lbl_state_value.configure(text="DISPENSE")
            self.dispenser_state_msg.request_guid_queue.append(msg.request_guid)

            # TODO: I actually want to see if the thread is joinable but 
            # apparently that's not how you do it in python ... whatever
            self.dispenser_frame()
            self.button_blinker_thread = Thread(target=self.blink_user_button_dispenser)
            if self.button_blinker_thread.is_alive():
                self.button_blinker_thread.join()

            self.button_blinker_thread.start()
        self.dispenser_state_mutex.release()

    def user_button_click_dispenser(self):
        self.dispenser_state_mutex.acquire()
        if self.dispenser_state_msg.mode == DispenserState().BUSY:
            # Set the dispenser temporarly offline, so that the "button blikning loop"
            # can be broken without accepting new requests
            self.dispenser_state_msg.mode = DispenserState().OFFLINE
            #self.lbl_state_value.configure(text="IDLE")
        self.dispenser_state_mutex.release()

    def blink_user_button_dispenser(self):
        print("starting to blink")
        while self.dispenser_state_msg.mode == DispenserState().BUSY:
            self.user_button3["background"] = "green2"
            self.user_button3["activebackground"] = "green2"
            sleep(1)

            self.user_button3["background"] = "green3"
            self.user_button3["activebackground"] = "green3"
            sleep(1)
        
        #print("blinking stopped")

        # Send a result to RMF
        dispenser_result_msg = DispenserResult()
        dispenser_result_msg.source_guid = self.dispenser_name
        dispenser_result_msg.status = DispenserResult().SUCCESS
        dispenser_result_msg.time = self.get_clock().now().to_msg()

        self.dispenser_state_mutex.acquire()
        dispenser_result_msg.request_guid = self.dispenser_state_msg.request_guid_queue.pop(0)
        self.add_served_dispenser_request(dispenser_result_msg.request_guid)
        self.dispenser_result_pub.publish(dispenser_result_msg)
        self.dispenser_state_msg.mode = DispenserState().IDLE
        self.dispenser_state_mutex.release()
        self.on_delivery_frame()

    def is_new_dispenser_request(self, request_guid):
        self.served_dispenser_requests_mutex.acquire()
        exists = request_guid in self.served_dispenser_requests
        self.served_dispenser_requests_mutex.release()
        return not exists

    def add_served_dispenser_request(self, request_guid):
        self.served_dispenser_requests_mutex.acquire()
        self.served_dispenser_requests.append(request_guid)
        self.served_dispenser_requests_mutex.release()

    def ingestor_state_timer_callback(self):
        self.ingestor_state_mutex.acquire()
        self.ingestor_state_msg.time = self.get_clock().now().to_msg()
        self.get_logger().debug('Publishing: "{0}"'.format(self.ingestor_state_msg))
        self.ingestor_state_pub.publish(self.ingestor_state_msg)
        self.ingestor_state_mutex.release()

    def ingestor_requests_callback(self, msg):
        print("received a ingestion request for '" + msg.request_guid + "'")
        self.ingestor_state_mutex.acquire()

        # Make sure that the ingestor is idling; the request is for this ingestor and 
        # this request has not been served before
        if self.ingestor_state_msg.mode == IngestorState().IDLE \
        and msg.target_guid == self.ingestor_name \
        and self.is_new_ingestor_request(msg.request_guid):
            self.ingestor_state_msg.mode = IngestorState().BUSY
            self.ingestor_state_msg.request_guid_queue.append(msg.request_guid)

            # TODO: I actually want to see if the thread is joinable but 
            # apparently that's not how you do it in python ... whatever
            self.ingestor_frame()
            self.button_blinker_ingestor_thread = Thread(target=self.blink_user_button_ingestor)
            if self.button_blinker_ingestor_thread.is_alive():
                self.button_blinker_ingestor_thread.join()

            self.button_blinker_ingestor_thread.start()
        self.ingestor_state_mutex.release()

    def user_button_click_ingestor(self):
        self.ingestor_state_mutex.acquire()
        if self.ingestor_state_msg.mode == IngestorState().BUSY:
            # Set the ingestor temporarly offline, so that the "button blikning loop"
            # can be broken without accepting new requests
            self.ingestor_state_msg.mode = IngestorState().OFFLINE
        self.ingestor_state_mutex.release()

    def blink_user_button_ingestor(self):
        print("starting to blink")
        while self.ingestor_state_msg.mode == IngestorState().BUSY:
            self.user_button6["background"] = "green2"
            self.user_button6["activebackground"] = "green2"
            sleep(1)

            self.user_button6["background"] = "green3"
            self.user_button6["activebackground"] = "green3"
            sleep(1)
        
        #print("blinking stopped")

        # Send a result to RMF
        ingestor_result_msg = IngestorResult()
        ingestor_result_msg.source_guid = self.ingestor_name
        ingestor_result_msg.status = IngestorResult().SUCCESS
        ingestor_result_msg.time = self.get_clock().now().to_msg()

        self.ingestor_state_mutex.acquire()
        ingestor_result_msg.request_guid = self.ingestor_state_msg.request_guid_queue.pop(0)
        self.add_served_ingestor_request(ingestor_result_msg.request_guid)
        self.ingestor_result_pub.publish(ingestor_result_msg)
        self.ingestor_state_msg.mode = IngestorState().IDLE
        self.ingestor_state_mutex.release()

        self.go_back_to_start()

    def is_new_ingestor_request(self, request_guid):
        self.served_ingestor_requests_mutex.acquire()
        exists = request_guid in self.served_ingestor_requests
        self.served_ingestor_requests_mutex.release()
        return not exists

    def add_served_ingestor_request(self, request_guid):
        self.served_ingestor_requests_mutex.acquire()
        self.served_ingestor_requests.append(request_guid)
        self.served_ingestor_requests_mutex.release()
        
    def show(self):
        self.root.mainloop()

def main(argv=sys.argv):
    # Init ros
    rclpy.init(args=sys.argv)

    # Get the name of the dispenser via arguments
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-dn', '--dispenser_name', required=True, type=str, help='Name of the dispenser')
    parser.add_argument('-in', '--ingestor_name', required=True, type=str, help='Name of the ingestor')
    parser.add_argument('-dl', '--dispenser_location', required=True, type=str, help='Location of the dispenser')
    parser.add_argument('-il', '--ingestor_location', required=True, type=str, help='Location of the ingestor')
    args = parser.parse_args(argv[1:])

    gui = DispenserIngestorGui(args.dispenser_name, args.dispenser_loaction, args.ingestor_name, args.ingestor_location)

    # I wasn't able to find an async spinner, thus starting the spinner in its own thread
    spin_thread = Thread(target=rclpy.spin, args=[gui])
    spin_thread.start()

    # Blocking call
    gui.show()

    rclpy.shutdown()
    gui.destroy_node()
    spin_thread.join()

if __name__ == '__main__':
    main(sys.argv)
