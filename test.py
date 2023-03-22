import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
# URI to the Crazyflie to connect to
fly = uri_helper.uri_from_env(default='radio://0/79/2M/E7E7E7E7E6')
track = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print('pos: ({}, {}, {})'.format(x, y, z))

def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=500)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def posOut( scf, lg_stab):
    with SyncLogger(scf, lg_stab) as logger:
        for log_entry in logger:
            print(log_entry[1])
            for j in log_entry[1].values():
                print(j)
            data = list(log_entry[1])
            print(data)



print(1)
cflib.crtp.init_drivers()
with SyncCrazyflie(track, cf=Crazyflie(rw_cache='./cache')) as tscf:
    print(4)
    log_conf = LogConfig(name='Position', period_in_ms=50)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    while True:
        posOut(tscf, log_conf)
    # with SyncCrazyflie(fly, cf=Crazyflie(rw_cache='./cache')) as fscf:
    #     print(2)
    #     with PositionHlCommander(fscf, controller=PositionHlCommander.CONTROLLER_PID) as fpc:
    #         print(3)
    #         fpc.go_to(.7,.6-1,1.2)
    #         while True:
    #             x=1
    #             y=1
    #             z=1
    #             if z<.2:
    #                 z=.2
    #             elif 1.3<z:
    #                 z= 1.3
    #             if x<-.7:
    #                 x=-.7
    #             elif .7<x:
    #                 x= .7
    #             if y<-.4:
    #                 y=-.4
    #             elif .6<y:
    #                 y= .6
    #             fpc.go_to(x,y,z)