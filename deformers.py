import numpy as np

"""
A class for handling the cacheing and parsing of signals to send to the 'deformer' remote controlled car
"""

# More than meets the eye!
class Deformer:
    def __init__(self, fs, instructions, times, buff_len=100000):
        self.move_instr = None
        self.move_cache = None
        self.header_move_cache = None

        self.fs = int(fs)
        self.instructions = instructions
        self.times = times
        self.buff_len = buff_len
        self.wave_memory = np.array([]).astype(np.complex64)
        self.max_memory = int(self.fs * 0.25)    # the maximum wave memory to store
        self.wave_buffer = np.zeros(shape=self.buff_len)
        self.freq = 1000   # Some random frequency for making the wave
        self.min_xmit_dur = 0.2

        # pulse widths
        self.sp = 2077/4e6*self.fs
        self.lp = 5701/4e6*self.fs
        self.sg = 2075/4e6*self.fs
        self.lg = 2325/4e6*self.fs

        # Headers, stored in memory for efficiency
        # Forward/backward
        self.fb_header = np.array([self.sp, self.lp, self.sp, self.lp]).repeat(4)

        # left/right
        self.lr_header = np.array([self.lp]).repeat(12)

        # DEFORMMMMMM!!!!!!!
        self.d_header = np.array([self.lp, self.lp,
                                  self.sp, self.sp, self.sp, self.sp,
                                  self.lp, self.lp, self.lp, self.lp,
                                  self.sp, self.sp, self.sp, self.sp,
                                  self.lp, self.lp, self.lp, self.lp])


        self.fb_header = self.add_gaps(message=self.fb_header).astype(np.int32)
        self.lr_header = self.add_gaps(message=self.lr_header).astype(np.int32)
        self.d_header = self.add_gaps(message=self.d_header).astype(np.int32)

        # Cache all the moves
        self.cache_moves()
        # Process any given instructions
        if len(self.instructions) > 0:
            self.process_moves()

    def cache_moves(self):
        """
        Caches all the moves for call during runtime
        """
        print("Caching move instructions")
        # Create the move instructions
        self.move_instr = {
            # Forwards
            'f': np.concatenate([np.array([self.sp]).repeat(28), np.array([self.lp]).repeat(4)]),
            'fr': np.concatenate([np.array([self.sp]).repeat(16), np.array([self.lp]).repeat(4)]),
            'fl': np.concatenate([np.array([self.sp]).repeat(58), np.array([self.lp]).repeat(4)]),
            # Backwards
            'b': np.concatenate([np.array([self.sp]).repeat(46), np.array([self.lp]).repeat(4)]),
            'br': np.concatenate([np.array([self.sp]).repeat(64), np.array([self.lp]).repeat(4)]),
            'bl': np.concatenate([np.array([self.sp]).repeat(40), np.array([self.lp]).repeat(4)]),
            # Left
            'l': np.concatenate([np.array([self.sp]).repeat(52), np.array([self.lp]).repeat(4)]),
            # Right
            'r': np.concatenate([np.array([self.sp]).repeat(34), np.array([self.lp]).repeat(4)]),
            # DEFORM
            'd': np.concatenate([np.array([self.sp]).repeat(22), np.array([self.lp]).repeat(4)])
        }

        # Add gaps into the moves
        for key in self.move_instr.keys():
            self.move_instr[key] = self.add_gaps(self.move_instr[key]).astype(np.int32)

        # Use the instructions to create the waves
        self.move_cache = {}
        for key in self.move_instr.keys():
            self.move_cache[key] = self.parse_moves(self.move_instr[key], header=np.array([]), t=self.min_xmit_dur)

        # Create the header moves
        self.header_move_cache = {}
        for key in self.move_instr.keys():
            if 'l' in key or 'r' in key or 'd' in key:
                head = self.lr_header
            else:
                head = self.fb_header
            self.header_move_cache[key] = self.parse_moves(self.move_instr[key], head, self.min_xmit_dur)

    def parse_moves(self, direction, header, t):
        """
        Returns the waveform for moving t seconds in the given direction. If there is no header just input an empty
        array
        """
        try:
            header_dur = header.sum() / self.fs
            body_dur = direction.sum() / self.fs
        except NameError:
            raise ValueError("direction not recognised")

        # Figure out how many times we are transmitting
        body_time = t - header_dur
        n = body_time / body_dur
        n = round(n, 0)
        n = max(1, n)   # clamp it to always be 1 or more
        n = int(n)

        # Create the message
        message = np.concatenate([header, np.tile(direction, n)]).astype(np.int32)

        # Create the wave
        wave_dur = message.sum() / self.fs
        z = self.create_wave(wave_dur)

        # Convert message to slices
        cs = np.cumsum(message)
        message_slices = tuple(slice(cs[i], cs[i+1]) for i in range(len(cs)-1))

        # Use the slices to zero out the gaps in the message
        for i in range(len(message_slices)):
            if not (i % 2): # get every second slice
                z[message_slices[i]] *= 0

        # Return the wave
        return z.astype(np.complex64)

    def move(self, direction, header=False):
        """
        Selects the correct move from the cached moves
        """
        if header:
            z = self.move_cache[direction]
        else:
            z = self.header_move_cache[direction]

        if len(self.wave_memory) > self.max_memory:
            self.wave_memory = z
        else:
            self.wave_memory = np.concatenate([self.wave_memory, z])

    def buffdate(self):
        """
        buffer update
        """
        if self.wave_memory.shape[0] > 0:
            self.wave_buffer = self.wave_memory[0:self.buff_len]
            self.wave_memory = self.wave_memory[self.buff_len:]
        else:
            self.wave_buffer = np.zeros(self.buff_len, dtype=np.complex64)
    def process_moves(self):
        """
        Processes the moves and times in the given lists and appends them to the queue
        """
        if len(self.instructions) != len(self.times):
            raise RuntimeError("Number of instructions is not equal to the possible number of times")

        for i in range(len(self.instructions)):
            self.move(self.instructions[i])

    def add_gaps(self, message):
        """
        Adds in the appropriate gaps to the header and instruction message items
        """
        out = []
        for i in range(len(message)):
            out.append(message[i])
            # If we're on the last one
            if i == len(message) - 1:
                out.append(self.sg)

            # If the next message is a short pulse
            elif message[i+1] == self.sp:
                out.append(self.sg)

            # if next message is a long pulse
            elif message[i+1] == self.lp:
                out.append(self.lg)

        return np.array(out)

    def create_wave(self, wave_time):
        """
        Creates a continuous wave that lasts for some duration
        """
        t = 1 / self.fs * np.arange(wave_time * self.fs)
        angle = 2 * np.pi * self.freq * t
        z = np.cos(angle) + 1j * np.sin(angle)
        z = z.astype(np.complex64)

        return z





