from deformers import Deformer
from pynput.keyboard import Key, Listener
from threading import Thread
import numpy as np
import time

"""
A class for handling input and output of signal data. Signals are selected based upon user keyboard inputs and then
added to memory. This memory is then shuttled into a buffer which is fed into a software defined radio for transmit
"""

class Drive(Deformer):
    def __init__(self, fs):
        super().__init__(fs, instructions=[], times=[])
        self.driving = False

        self.commands = np.array([
            ['hf','f'],
            ['hb', 'b'],
            ['hl', 'l'],
            ['hr', 'r'],
            ['hd', 'd']
        ])
        # rows are f, b, l, r, deform
        # Columns are previous state, current state
        self.state = np.zeros(shape=(5, 2), dtype=np.int8)
        self.last_cmd_time = time.time()
        self.polling_frequency = 0.1

    def on_press(self, key):

        try:
            # Pressing esc closes the program
            if key == key.esc:
                return False
            elif key == key.up:
                # Turn forward on, turn backwards off
                self.state[0, 1] = 1
                self.state[1, 1] = 0

            elif key == key.down:
                # Turn forwads off turn backwards on
                self.state[0, 1] = 0
                self.state[1, 1] = 1

            elif key == key.left:
                # Turn left on, turn right off
                self.state[2, 1] = 1
                self.state[3, 1] = 0

            elif key == key.right:
                # Turn left off, turn right on
                self.state[2, 1] = 0
                self.state[3, 1] = 1
            elif key == key.space:
                # Turn deform on turn everything else off
                self.state[4, 1] = 1
                self.state[0:4, 1] = 0

        # If the key is one that hasn't been programmed
        except AttributeError:
            print("Unknown key")

    def on_release(self, key):
        try:
            if key == key.esc:
                return False
            elif key == key.up:
                self.state[0, 1] = 0
            elif key == key.down:
                self.state[1, 1] = 0
            elif key == key.left:
                self.state[2, 1] = 0
            elif key == key.right:
                self.state[3, 1] = 0
            elif key == key.space:
                self.state[4, 1] = 0
        except AttributeError:
            print("Unknown key")

    def parse_state(self):
        mask = self.state == 1
        commands = self.commands[mask]

        # Forwards and right
        if 'f' in commands and 'r' in commands:
            # If the header fields are in the commands then we DONT need to send the header
            # If they are not in the commands then we DO need to send the header
            head_bool = 'hf' not in commands or 'hr' not in commands
            self.move(direction='fr', header=head_bool)

        # Forwards and left
        elif 'f' in commands and 'l' in commands:
            head_bool = 'hf' not in commands or 'hl' not in commands
            self.move(direction='fl', header=head_bool)

        # Just forwards
        elif 'f' in commands:
            head_bool = 'hf' not in commands
            self.move(direction='f', header=head_bool)

        # Back left
        elif 'b' in commands and 'l' in commands:
            head_bool = 'hb' not in commands or 'hl' not in commands
            self.move(direction='bl', header=head_bool)

        # Back right
        elif 'b' in commands and 'r' in commands:
            head_bool = 'hb' not in commands or 'hr' not in commands
            self.move(direction='br', header=head_bool)

        # Back
        elif 'b' in commands:
            head_bool = 'hb' not in commands
            self.move(direction='b', header=head_bool)

        # Left
        elif 'l' in commands:
            head_bool = 'hl' not in commands
            self.move(direction='l', header=head_bool)

        # Right
        elif 'r' in commands:
            head_bool = 'hr' not in commands
            self.move(direction='r', header=head_bool)

        # DEFORRRM
        elif 'd' in commands:
            head_bool = 'hd' not in commands
            self.move(direction='d', header=head_bool)

        # Now store the previous command in the header column
        self.state[:, 0] = self.state[:, 1]
        # Zero out the current command???
        #self.state[:, 1] = 0


    def move_poll(self):
        while self.driving:
            now = time.time()
            if (now - self.last_cmd_time) < self.polling_frequency:
                pass
            else:
                self.last_cmd_time = now
                self.parse_state()

    def run(self):
        self.driving = True

        # Start the polling of the keyboard (which checks the current state of button presses)
        keyboard_poll = Thread(target=self.move_poll)
        keyboard_poll.start()


        # Start the keyboard listener, this blocks new threads until esc is pressed
        with Listener(
                on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

        self.driving = False
        keyboard_poll.join()


