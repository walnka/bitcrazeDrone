import time
import math
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.commander import Commander
from cflib.utils import uri_helper

# URI to the Crazyflie to connect to
fly = uri_helper.uri_from_env(default='radio://0/79/2M/E7E7E7E7E6')
track = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')


cflib.crtp.init_drivers()
with SyncCrazyflie(fly, cf=Crazyflie(rw_cache='./cache')) as fscf:
    with Commander(fscf) as fpc:
                fpc.go_to(.7,.6-1,1.2)
                while True:
                    z=1+.1
                    y=1
                    x=1
                    if z<.1:
                        z=.1
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
                    fpc.go_to(x,y,z)
