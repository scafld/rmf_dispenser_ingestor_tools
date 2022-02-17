from tkinter import *
from threading import Thread, Lock
from time import sleep
import sys
import argparse

import rclpy

class DispenserTestGui:
    def __init__(self, dispenser_name):
        self.dispenser_name = dispenser_name
        self.window = Tk()
        self.window.title("Manual dispenser: " + dispenser_name)
        self.window.geometry('350x200')

        # Initialize the buttons
        self.test_button = Button(self.window, text="TEST", command=self.test_button_click)
        self.test_button.grid(column=0, row=3)

        self.user_button = Button(self.window, text="", command=self.user_button_click)
        self.user_button.grid(column=0, row=4)

        # Initialize the labels for dispenser state feedback
        self.state = "IDLE"
        self.lbl_state_name = Label(self.window, text="State: ")
        self.lbl_state_name.grid(column=0, row=0)

        self.lbl_state_value = Label(self.window, text=self.state)
        self.lbl_state_value.grid(column=1, row=0)

        # Create a thread for blinking the button
        self.blink_the_button = False

    def user_button_click(self):
        if self.state == "BLINKING":
            self.state = "IDLE"
            self.lbl_state_value.configure(text=self.state)
            self.blink_the_button = False

    def test_button_click(self):
        if self.state == "IDLE":
            self.state = "BLINKING"
            self.lbl_state_value.configure(text=self.state)
            self.button_blinker_thread = Thread(target=self.blink_user_button)

            # TODO: I actually want to see if the thread is joinable but 
            # apparently that's not how you do it in python ... whatever
            if self.button_blinker_thread.is_alive():
                self.button_blinker_thread.join()

            self.button_blinker_thread.start()

    def blink_user_button(self):
          self.blink_the_button = True
          print("starting to blink")

          while self.blink_the_button:
              self.user_button["background"] = "yellow"
              self.user_button["activebackground"] = "yellow"
              sleep(1)

              self.user_button["background"] = "gray"
              self.user_button["activebackground"] = "gray"
              sleep(1)

    def show(self):
        self.window.mainloop()

def main(argv=sys.argv):
    args_without_ros = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--name', required=True, type=str, help='Name of the dispenser')
    args = parser.parse_args(argv[1:])
    
    dispenser_test_gui = DispenserTestGui(args.name)
    dispenser_test_gui.show()