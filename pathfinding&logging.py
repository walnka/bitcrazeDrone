import logging
import time
import numpy as np
import math

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

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

def kahanP1(u,v):
    u = u/np.linalg.norm(u)
    v = v/np.linalg.norm(v)
    return 2*math.atan(np.linalg.norm(u-v)/np.linalg.norm(u+v)) 

def oobounds(tar_pos):
    x = tar_pos[0]
    y = tar_pos[1]
    z = tar_pos[2]
    if z<.2:
        z=.2
    elif 1.3<z:
        z= 1.3
    if x<-.7:
        x=-.7
    elif .7<x:
        x= .7
    if y<-.4:
        y=-.4
    elif .6<y:
        y= .6
    return np.array([x,y,z])

# Finds next location for pursuer to go. 
# Run next line to call function in main
# pursue(fpc, lg_stabf.data, lg_stabt.data)
def pursue(pc, fdata, tdata):
    T_pos=np.array(tdata[0:3])
    T_quat=np.array(tdata[3:7])
    F_pos=np.array(fdata[0:3])
    # calculates the vector
    diff=F_pos-T_pos
    #calculates the yaw of the tracking drone should be the same as the above yaw within error
    T_yaw=math.atan2(2*(T_quat[3]*T_quat[0]+T_quat[1]*T_quat[2]),-(1-2*(T_quat[0]**2+T_quat[1]**2)))
    #calculates the final drone position that is above and behind the tracking drone
    Final_pos=T_pos+tar_rad/math.sqrt(2)*np.array([-math.cos(T_yaw),-math.sin(T_yaw),1])
    #calculates the straight path vector between the flying drone and the target position
    diff_F=Final_pos-F_pos
    #finds the vector that has the shortest distance between the tracking drone and the straight line path of the flying drone to its target position
    rad_vec=diff-(np.dot(diff,diff_F)/np.dot(diff_F,diff_F)*diff_F)
    act_rad = np.linalg.norm(rad_vec)
    #current radius from the tracking drone
    cur_rad=np.linalg.norm(diff)

    if np.linalg.norm(diff)<min_rad:
        #if the flying drone is currently too close to the tracking drone it will move away in the shortest path until it can use another pathing method to get around
        print("Too close")
        Tar_pos=T_pos+diff/cur_rad*tar_rad
    elif kahanP1(T_pos-Final_pos,F_pos-Final_pos)<math.asin(min_rad/tar_rad):
        #if the path goes to close to the tracking drone it creates a path that goes around the drone. not the shortest path but will go around the tracking drone in the shortest direction
        print("Intersecting")
        Tar_pos=T_pos+rad_vec*(act_rad+tar_rad)/act_rad
    else:
        Tar_pos=Final_pos
    gotoLoc(pc,Tar_pos, T_yaw, 0.5)

# Function to go to location, used in pursuer and tracker
def gotoLoc(pc,pos,yaw,v):
    Tar_pos = oobounds(pos)
    x = Tar_pos[0]
    y = Tar_pos[1]
    z = Tar_pos[2]
    # print(Tar_pos)
    # print(Tar_yaw)
    pc.go_to(x,y,z,yaw,v)


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
    with SyncCrazyflie(urit, cf=Crazyflie(rw_cache='./cachet')) as tscf:
        with SyncCrazyflie(urif, cf=Crazyflie(rw_cache='./cachef')) as fscf:
            with PositionHlCommander(fscf, controller=PositionHlCommander.CONTROLLER_PID) as fpc:
                with PositionHlCommander(tscf, controller=PositionHlCommander.CONTROLLER_PID) as tpc:
                    simple_log_async_start(tscf, lg_stabt)
                    simple_log_async_start(fscf, lg_stabf)
                    time.sleep(1)
                    # TODO: Change below code to FSM for diifferent conditions to show off features of our code
                    # See nextsteps.txt for more info
                    while True:
                        pursue(fpc, lg_stabf.data, lg_stabt.data)
                        time.sleep(0.01)
