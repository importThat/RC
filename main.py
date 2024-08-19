import uhd
from threading import Thread
from drive import Drive
from time import sleep

"""
This program takes in user keyboard input and transmits signals based on the input to a USRP. Order of operations are
as follows:
    1 - The USRP particulars are set up (sampling frequency, operating frequency etc) and the USRP is initialised as a
        stream
    2 - Waves are created and cached by the Drive class, which is a child of the Deformer class. This occurs on line 41.
    3 - Threads are started and a blocking pynput.Listener function executes until the user presses esc
    
Once blocking has begun, press the up arrow to direct the car up, left to turn it left, right to go right etc.
Pressing up and right will send the signal to do an arced turn forward and right. Pressing space bar sends the DEFORM
command.

The input works by using pynput to monitor the keyboard. Valid keyboard presses change an internal state of the Drive
class, which is polled every 0.1 seconds. Based upon the state, the polling process then puts the appropriate signal
into memory, which is read into the USRP transmit buffer when it is emptied.

If you are seeing a lot of red 'U' being printed in the terminal, that means that the buffer size needs to be raised.
This is controlled by the buff_len argument in the Deformer class
"""

def stream_xmit(streamer, metadata, drive):
    while drive.driving:
        # Send samps number of samples to the usrp for xmit
        samps = streamer.send(drive.wave_buffer, metadata)

        # update the buffer
        drive.buffdate()

sample_rate = 4e6
freq = 40398999 - 1000 # The driver freq
gain = 90

# Setup tx options
usrp = uhd.usrp.MultiUSRP()
usrp.set_tx_rate(sample_rate, 0)
usrp.set_tx_freq(uhd.libpyuhd.types.tune_request(freq), 0)
usrp.set_tx_gain(gain, 0)

# Setup the stream
stream_args = uhd.usrp.StreamArgs("fc32", "sc16")
stream_args.args = "spp=200"    # samples per packet
tx_streamer = usrp.get_tx_stream(stream_args)
tx_metadata = uhd.types.TXMetadata()

# Initialise the driver
driver = Drive(fs=sample_rate)

# create the threads
t1 = Thread(target=driver.run)
t2 = Thread(target=stream_xmit, args=(tx_streamer, tx_metadata, driver))

# Start the threads, gotta start the driver first
t1.start()
sleep(0.1)
t2.start()

t1.join()
t2.join()
