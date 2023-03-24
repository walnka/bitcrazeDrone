import logging
import time
import numpy as np
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
# URI to the Crazyflie to connect to
urit = 'radio://0/80/2M/E7E7E7E7E7'
urif = 'radio://0/79/2M/E7E7E7E7E6'
min_rad=.15
tar_rad=.2
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    logconf.data=list(data.values())

def simple_log_async_start(scf, logconf):
    logconf.data=[]
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

def simple_log_async_stop(scf, logconf):
    logconf.stop()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    lg_stabt = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stabt.add_variable('kalman.stateX', 'FP16')
    lg_stabt.add_variable('kalman.stateY', 'FP16')
    lg_stabt.add_variable('kalman.stateZ', 'FP16')
    lg_stabt.add_variable('kalman.q0', 'float')
    lg_stabt.add_variable('kalman.q1', 'float')
    lg_stabt.add_variable('kalman.q2', 'float')
    lg_stabt.add_variable('kalman.q3', 'float')
    lg_stabf = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stabf.add_variable('kalman.stateX', 'FP16')
    lg_stabf.add_variable('kalman.stateY', 'FP16')
    lg_stabf.add_variable('kalman.stateZ', 'FP16')
    with SyncCrazyflie(urit, cf=Crazyflie(rw_cache='./cache')) as tscf:
        with SyncCrazyflie(urif, cf=Crazyflie(rw_cache='./cache')) as fscf:
            simple_log_async_start(tscf, lg_stabt)
            simple_log_async_start(fscf, lg_stabf)
            time.sleep(1)
            while True:
                T_pos=np.array(lg_stabt.data[0:3])
                T_quat=np.array(lg_stabt.data[3:7])
                F_pos=np.array(lg_stabf.data[0:3])
                # calculates the vector
                diff=F_pos-T_pos
                #calculates the target yaw for the flying drone to be looking at the tracking drone
                F_yaw=math.atan(diff[1]/diff[0])
                #calculates the yaw of the tracking drone should be the same as the above yaw within error
                T_yaw=math.atan(2*(T_quat[3]*T_quat[0]+T_quat[1]*T_quat[2])/(1-2*(T_quat[0]**2+T_quat[1]**2)))
                #calculates the final drone position that is above and behind the tracking drone
                Final_pos=T_pos+tar_rad*np.array([math.sin(T_yaw),math.cos(T_yaw),1])
                #calculates the straight path vector between the flying drone and the target position
                diff_F=Final_pos-F_pos
                #finds the vector that has the shortest distance between the tracking drone and the straight line path of the flying drone to its target position
                rad_vec=diff-(np.dot(diff,diff_F)/np.dot(diff_F,diff_F)*diff_F)
                #this gives us the distance between the target path and the straight path
                act_rad=np.linalg.norm(rad_vec)
                #current radius from the tracking drone
                cur_rad=np.linalg.norm(diff)
                
                if np.linalg.norm(diff)<min_rad:
                    #if the flying drone is currently too close to the tracking drone it will move away in the shortest path until it can use another pathing method to get around
                    Tar_pos=T_pos+diff/cur_rad*tar_rad
                elif act_rad<min_rad:
                    #if the path goes to close to the tracking drone it creates a path that goes around the drone. not the shortest path but will go around the tracking drone in the shortest direction
                    Tar_pos=T_pos+rad_vec*(act_rad+tar_rad)/act_rad
                else:
                    Tar_pos=Final_pos
                Tar_yaw=F_yaw
                print(Tar_pos)
                print(Tar_yaw)