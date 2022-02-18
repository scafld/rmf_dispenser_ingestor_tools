from tkinter import *
from threading import Thread, Lock
from time import sleep
import sys
import argparse

import rclpy
from rclpy.node import Node
from rmf_dispenser_msgs.msg import DispenserState
from rmf_dispenser_msgs.msg import DispenserRequest
from rmf_dispenser_msgs.msg import DispenserResult

class DispenserTestGui(Node):
    def __init__(self, dispenser_name):
        self.dispenser_name = dispenser_name
        super().__init__(self.dispenser_name)

        self.window = Tk()
        self.window.title("Manual dispenser: " + dispenser_name)
        self.window.geometry('500x500')

        # Initialize the labels for dispenser state feedback
        self.horizontal_frame = Frame(self.window)
        self.horizontal_frame.pack()
        self.lbl_state_name = Label(self.horizontal_frame, text="State: ")
        self.lbl_state_value = Label(self.horizontal_frame, text="IDLE")

        # Initialize the buttons
        self.user_button = Button(self.window, text="", command=self.user_button_click, height = 20, width = 30)
        # self.user_button.grid(column=0, row=4)

        self.lbl_state_name.pack(side=LEFT)
        self.lbl_state_value.pack(side=LEFT)
        self.user_button.pack()

        # Initialize the dispenser state
        self.dispenser_state_msg = DispenserState()
        self.dispenser_state_msg.guid = self.dispenser_name
        self.dispenser_state_msg.mode = DispenserState().IDLE
        self.dispenser_state_mutex = Lock()

        # Set up publishers, subscribers, etc
        self.dispenser_state_timer = self.create_timer(2.0, self.dispenser_state_timer_callback)
        self.dispenser_state_pub = self.create_publisher(DispenserState, 'dispenser_states', 1)
        self.dispenser_result_pub = self.create_publisher(DispenserResult, 'dispenser_results', 1)
        self.dispenser_request_sub = self.create_subscription(DispenserRequest, "dispenser_requests", self.dispenser_requests_callback, 1)

        self.get_logger().info('The dispenser is good to go')

    def dispenser_state_timer_callback(self):
        self.dispenser_state_mutex.acquire()
        self.dispenser_state_msg.time = self.get_clock().now().to_msg()
        self.get_logger().debug('Publishing: "{0}"'.format(self.dispenser_state_msg))
        self.dispenser_state_pub.publish(self.dispenser_state_msg)
        self.dispenser_state_mutex.release()

    def dispenser_requests_callback(self, msg):
        print("received a dispensing request")
        self.dispenser_state_mutex.acquire()
        if self.dispenser_state_msg.mode == DispenserState().IDLE and msg.target_guid == self.dispenser_name:
            self.dispenser_state_msg.mode = DispenserState().BUSY
            self.lbl_state_value.configure(text="DISPENSE")
            self.dispenser_state_msg.request_guid_queue.append(msg.request_guid)

            # TODO: I actually want to see if the thread is joinable but 
            # apparently that's not how you do it in python ... whatever
            self.button_blinker_thread = Thread(target=self.blink_user_button)
            if self.button_blinker_thread.is_alive():
                self.button_blinker_thread.join()

            self.button_blinker_thread.start()
        self.dispenser_state_mutex.release()

    def user_button_click(self):
        self.dispenser_state_mutex.acquire()
        if self.dispenser_state_msg.mode == DispenserState().BUSY:
            self.dispenser_state_msg.mode = DispenserState().IDLE
            self.lbl_state_value.configure(text="IDLE")
        self.dispenser_state_mutex.release()

    def blink_user_button(self):
        self.blink_the_button = True
        print("starting to blink")

        while self.dispenser_state_msg.mode == DispenserState().BUSY:
            self.user_button["background"] = "yellow"
            self.user_button["activebackground"] = "yellow"
            sleep(1)

            self.user_button["background"] = "gray"
            self.user_button["activebackground"] = "gray"
            sleep(1)
        
        print("blinking stopped")
        dispenser_result_msg = DispenserResult()
        dispenser_result_msg.source_guid = self.dispenser_name
        dispenser_result_msg.status = DispenserResult().SUCCESS

        self.dispenser_state_mutex.acquire()
        dispenser_result_msg.request_guid = self.dispenser_state_msg.request_guid_queue.pop(0)
        self.dispenser_state_mutex.release()
        self.dispenser_result_pub.publish(dispenser_result_msg)

    def show(self):
        self.window.mainloop()

def main(argv=sys.argv):
    rclpy.init(args=sys.argv)

    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', required=True, type=str, help='Name of the dispenser')
    args = parser.parse_args(argv[1:])    
    dispenser_test_gui = DispenserTestGui(args.name)

    # I wasn't able to find an async spinner, thus starting the spinner in its own thread
    spin_thread = Thread(target=rclpy.spin, args=[dispenser_test_gui])
    spin_thread.start()

    # Blocking call
    dispenser_test_gui.show()

    rclpy.shutdown()
    dispenser_test_gui.destroy_node()
    spin_thread.join()

if __name__ == '__main__':
    main(sys.argv)