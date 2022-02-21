from tkinter import *
from threading import Thread, Lock
from time import sleep
import sys
import argparse

import rclpy
from rclpy.node import Node
from rmf_ingestor_msgs.msg import IngestorState
from rmf_ingestor_msgs.msg import IngestorRequest
from rmf_ingestor_msgs.msg import IngestorResult

class IngestorTestGui(Node):
    def __init__(self, ingestor_name):
        self.ingestor_name = ingestor_name
        super().__init__(self.ingestor_name)

        self.window = Tk()
        self.window.title("Manual ingestor: " + ingestor_name)
        self.window.geometry('500x500')

        # Initialize the labels for ingestor state feedback
        self.horizontal_frame = Frame(self.window)
        self.horizontal_frame.pack()
        self.lbl_state_name = Label(self.horizontal_frame, text="State: ")
        self.lbl_state_value = Label(self.horizontal_frame, text="IDLE")

        # Initialize the buttons
        self.user_button = Button(self.window, text="", command=self.user_button_click, height = 20, width = 30)
        self.lbl_state_name.pack(side=LEFT)
        self.lbl_state_value.pack(side=LEFT)
        self.user_button.pack()

        # Initialize the ingestor state and list of served requests
        self.ingestor_state_msg = IngestorState()
        self.ingestor_state_msg.guid = self.ingestor_name
        self.ingestor_state_msg.mode = IngestorState().IDLE
        self.ingestor_state_mutex = Lock()
        self.served_requests_mutex = Lock()
        self.served_requests = []

        # Set up publishers, subscribers, etc
        self.ingestor_state_timer = self.create_timer(2.0, self.ingestor_state_timer_callback)
        self.ingestor_state_pub = self.create_publisher(IngestorState, 'ingestor_states', 1)
        self.ingestor_result_pub = self.create_publisher(IngestorResult, 'ingestor_results', 1)
        self.ingestor_request_sub = self.create_subscription(IngestorRequest, "ingestor_requests", self.ingestor_requests_callback, 1)

        self.get_logger().info('The ingestor is good to go')

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
        and self.is_new_request(msg.request_guid):
            self.ingestor_state_msg.mode = IngestorState().BUSY
            self.lbl_state_value.configure(text="DISPENSE")
            self.ingestor_state_msg.request_guid_queue.append(msg.request_guid)

            # TODO: I actually want to see if the thread is joinable but 
            # apparently that's not how you do it in python ... whatever
            self.button_blinker_thread = Thread(target=self.blink_user_button)
            if self.button_blinker_thread.is_alive():
                self.button_blinker_thread.join()

            self.button_blinker_thread.start()
        self.ingestor_state_mutex.release()

    def user_button_click(self):
        self.ingestor_state_mutex.acquire()
        if self.ingestor_state_msg.mode == IngestorState().BUSY:
            # Set the ingestor temporarly offline, so that the "button blikning loop"
            # can be broken without accepting new requests
            self.ingestor_state_msg.mode = IngestorState().OFFLINE
            self.lbl_state_value.configure(text="IDLE")
        self.ingestor_state_mutex.release()

    def blink_user_button(self):
        print("starting to blink")
        while self.ingestor_state_msg.mode == IngestorState().BUSY:
            self.user_button["background"] = "green"
            self.user_button["activebackground"] = "green"
            sleep(1)

            self.user_button["background"] = "gray"
            self.user_button["activebackground"] = "gray"
            sleep(1)
        
        print("blinking stopped")

        # Send a result to RMF
        ingestor_result_msg = IngestorResult()
        ingestor_result_msg.source_guid = self.ingestor_name
        ingestor_result_msg.status = IngestorResult().SUCCESS
        ingestor_result_msg.time = self.get_clock().now().to_msg()

        self.ingestor_state_mutex.acquire()
        ingestor_result_msg.request_guid = self.ingestor_state_msg.request_guid_queue.pop(0)
        self.add_served_request(ingestor_result_msg.request_guid)
        self.ingestor_result_pub.publish(ingestor_result_msg)
        self.ingestor_state_msg.mode = IngestorState().IDLE
        self.ingestor_state_mutex.release()

    def is_new_request(self, request_guid):
        self.served_requests_mutex.acquire()
        exists = request_guid in self.served_requests
        self.served_requests_mutex.release()
        return not exists

    def add_served_request(self, request_guid):
        self.served_requests_mutex.acquire()
        self.served_requests.append(request_guid)
        self.served_requests_mutex.release()

    def show(self):
        self.window.mainloop()

def main(argv=sys.argv):
    # Init ros
    rclpy.init(args=sys.argv)

    # Get the name of the ingestor via arguments
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', required=True, type=str, help='Name of the ingestor')
    args = parser.parse_args(argv[1:])

    # Initialize the ingestor
    ingestor_test_gui = IngestorTestGui(args.name)

    # I wasn't able to find an async spinner, thus starting the spinner in its own thread
    spin_thread = Thread(target=rclpy.spin, args=[ingestor_test_gui])
    spin_thread.start()

    # Blocking call
    ingestor_test_gui.show()

    rclpy.shutdown()
    ingestor_test_gui.destroy_node()
    spin_thread.join()

if __name__ == '__main__':
    main(sys.argv)