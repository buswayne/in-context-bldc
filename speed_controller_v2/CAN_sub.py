import os
import ctypes
import numpy as np  

from pathlib import Path
import time
import torch
import math
import gc
from transformer_zerostep import GPTConfig, GPT, warmup_cosine_lr

import can
import uptime
import matplotlib.pyplot as plt
import struct

import pandas as pd



    
class FilteredListener(can.Listener):
    def __init__(self, bus):

        self.time_log = []
        self.bus=bus
        

    
    def on_message_received(self, msg):

        if msg.arbitration_id == 0x103:
            self.process_time_data(msg)
            
    def process_time_data(self,msg):
        data = msg.data
        time_counter = (
            (data[0]) |
            (data[1] << 8) |
            (data[2] << 16) |
            (data[3] << 24)
        )
        # print(time_counter)
        # print(time_counter/1e6)
        self.time_log.append(time_counter)


    

def main():


    filters = [
    {"can_id": 0x101, "can_mask": 0x7FF, "extended": False},  # Standard ID 0x101
    {"can_id": 0x102, "can_mask": 0x7FF, "extended": False},  # Standard ID 0x102
    {"can_id": 0x103, "can_mask": 0x7FF, "extended": False},  # Standard ID 0x103
    {"can_id": 0x321, "can_mask": 0x7FF, "extended": False},  # Standard ID 0x321
    {"can_id": 0x333, "can_mask": 0x7FF, "extended": False},  # Standard ID 0x333
    ]
    
    with can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000) as bus:
        bus.set_filters(filters)
        # listener = FilteredListener(target_id)
        listener = FilteredListener(bus=bus)
        notifier = can.Notifier(bus, [listener])
        
        try:
            # print(f"Listening for messages with IDs: {hex(target_id)}")
            print("listening for timer: ")
            print("Press Ctrl+C to stop...")
            start = time.time()
            max_time = np.inf
            while time.time() - start < max_time:
                # Keep the main thread alive
                can.BufferedReader().get_message(timeout=1)
        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            notifier.stop()
            time_log = np.array(listener.time_log)
            
            
            print(f"received {len(time_log)} messages")
            mask = time_log/1e6<1
            time_log_filt = time_log[mask]


            fig = plt.figure()
            ax0 = fig.add_subplot(1,1,1)
            ax0.plot(time_log/1e6)
            ax0.set_ylabel("dt [s]")
            lims = ax0.get_xlim()

            fig = plt.figure()
            ax0 = fig.add_subplot(1,1,1)
            window_size = 300 #circa 3s
            time_series = np.array(pd.Series(time_log_filt/1e3).rolling(window=window_size).mean())
            ax0.plot(time_series)
            ax0.set_ylabel("dt [ms]")

            # mask = ~np.isnan(time_series)
            print(np.nanargmax(time_series))
            max_movmean_idx = np.nanargmax(time_series)
            hotspot = time_log_filt[max_movmean_idx-int(window_size/2):max_movmean_idx+int(window_size/2)]

            fig = plt.figure()
            ax0 = fig.add_subplot(1,1,1)
            ax0.plot(np.arange(len(time_log_filt))[max_movmean_idx-int(window_size/2):max_movmean_idx+int(window_size/2)], hotspot)
            ax0.set_ylabel("dt [s]")
            ax0.set_xlim(lims)

            print(f"average computation time: {time_log.mean()/1e3} ms")
            print(f"std computation time: {(time_log/1e3).std()} ms")
            print(f"filt average computation time: {time_log_filt.mean()/1e3} ms")
            print(f"filt std computation time: {(time_log_filt/1e3).std()} ms")
            print(f"filt average computation time (hotspot): {hotspot.mean()/1e3} ms")
            print(f"filt std computation time (hotspot): {(hotspot/1e3).std()} ms")
            plt.tight_layout()
            plt.show()


if __name__ == "__main__":
    main()