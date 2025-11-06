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


current_path = os.getcwd().split("speed_controller_v2")[0]

dll_dir = os.path.join(current_path,"speed_controller_v2", "C_libs")
os.add_dll_directory(dll_dir)
lib_path = os.path.join(dll_dir, "net_predict_L_40k_test_mo_openmp.dll")
lib = ctypes.CDLL(lib_path)

lib.net_predict_L_40k.restype = None
lib.net_predict_L_40k.argtypes = [
    np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags="C_CONTIGUOUS"),  # input
    np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags="C_CONTIGUOUS")   # output
]



    
class FilteredListener(can.Listener):
    def __init__(self, H, bus):
        # self.target_ids = set(target_ids)  # Convert to set for fast lookup
        # self.target_id = target_id
        self.H = H
        self.data_vector = np.zeros((1,H,6))
        self.startup = True
        self.output = np.zeros(H, dtype=np.float32)
        self.bus = bus
        self.msg = can.Message(arbitration_id=0x333, is_extended_id=False, dlc=8)

        self.time_log = []
        self.iq_log = []
        self.iq_ref_log = []

    
    def on_message_received(self, msg):
        # if msg.arbitration_id in self.target_ids:
        if msg.arbitration_id == 0x101:
            self.process_electric_data(msg)
            
        elif msg.arbitration_id == 0x102:
            self.process_speed_data(msg)

        elif msg.arbitration_id == 0x103:
            self.process_time_data(msg)
            

    def process_electric_data(self, msg):
        # msg.data.[...]
        # start = time.perf_counter_ns()
        data = msg.data

        # reconstruct 12-bit words (little-endian packing used by the C code)
        send_id  = ((data[1] & 0x0F) << 8) | data[0]                # bits [11:0] of id
        send_iq  = (data[2] << 4) | ((data[1] >> 4) & 0x0F)         # bits [11:0] of iq
        send_vd  = ((data[4] & 0x0F) << 8) | data[3]                # bits [11:0] of vd
        send_vq  = (data[5] << 4) | ((data[4] >> 4) & 0x0F)         # bits [11:0] of vq

        # inverse scaling (reverse of C encoding)
        id = (send_id / 204.75) - 10.0
        id_scaled = (id + 5) / 10
        iq = (send_iq / 204.75) - 10.0
        iq_scaled = (iq + 5) / 10
        vd = (send_vd / 68.25) - 30.0
        vd_scaled = (vd + 24) / 48
        vq = (send_vq / 68.25) - 30.0
        vq_scaled = (vq + 24) / 48


        self.data_vector[0, 0:self.H-1, 0:4] = self.data_vector[0, 1:self.H, 0:4]
        self.data_vector[0,self.H-1,0:4] = [id_scaled,iq_scaled,vd_scaled,vq_scaled]
        # print(f"it took {(time.perf_counter_ns()-start)*1e-9}s")
        # print(self.data_vector[0,:,:])
        # print([id,iq,vd,vq])
        self.iq_log.append(iq)
        

    def process_speed_data(self, msg):
        # msg.data.[...]
        # start = time.perf_counter_ns()
        data = msg.data

        # --- Decode omega (2 bytes, little-endian) ---
        send_omega = data[0] | (data[1] << 8)
        omega = (send_omega / 6.5535) - 5000.0  # inverse of encoding
        omega_scaled = omega / 2500

        # --- Decode omega_ref (2 bytes, little-endian) ---
        send_omega_ref = data[2] | (data[3] << 8)
        omega_ref = (send_omega_ref / 6.5535) - 5000.0
        omega_ref_scaled = omega_ref / 2500
        # print(omega_ref)

        # --- Decode time_counter (4 bytes, little-endian) ---
        time_counter = (
            (data[4]) |
            (data[5] << 8) |
            (data[6] << 16) |
            (data[7] << 24)
        )

        self.data_vector[0, 0:self.H-1, 4:6] = self.data_vector[0, 1:self.H, 4:6]
        self.data_vector[0,self.H-1,4:6] = [omega_scaled, omega_ref_scaled]

        if self.startup:
            self.data_vector[0,0:self.H-1] = self.data_vector[0,self.H-1]
            self.startup = False

        net_in = self.data_vector.astype(np.float64).flatten(order='F')
        lib.net_predict_L_40k(net_in, self.output)
        out = self.output
        iq_ref = out[-1] * 10 - 5
        self.iq_ref_log.append(iq_ref)

        iq_ref_send = max(-10.0, min(10.0, iq_ref))
        iq_ref_send_pack = struct.pack('<f', iq_ref_send)
        # print(iq_ref)
        # iq_ref_send = 
        self.msg.data = [data[4],data[5],data[6],data[7],
                         iq_ref_send_pack[0],iq_ref_send_pack[1],iq_ref_send_pack[2],iq_ref_send_pack[3]]
        # self.msg.data = [0,0,0,0,0,0,0,0]
        self.bus.send(self.msg)
        # print(f"it took {(time.perf_counter_ns()-start)*1e-9}s")
        # print(self.data_vector[0,:,:])
        # print(time_counter)
        # print(omega)

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

    for i in range(20):
        x_in = np.random.rand(60).astype(np.float64)  # your input array
        y_out = np.zeros(10, dtype=np.float32)        # output array to be filled
        lib.net_predict_L_40k(x_in, y_out)

    


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
        listener = FilteredListener(H=10, bus=bus)
        notifier = can.Notifier(bus, [listener])
        
        try:
            # print(f"Listening for messages with IDs: {hex(target_id)}")
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
            iq_log = np.array(listener.iq_log)
            iq_ref_log = np.array(listener.iq_ref_log)
            plt.figure()
            plt.plot(time_log/1e6)
            plt.figure()
            plt.plot(iq_ref_log, label="iq_ref")
            plt.plot(iq_log, label="iq")
            plt.legend()
            print(f"received {len(time_log)} messages")
            print(f"average delay: {time_log.mean()/1e6}")
            plt.show()


if __name__ == "__main__":
    main()