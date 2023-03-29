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
# URI for each of the drones
drone5='radio://0/78/2M/E7E7E7E7E5'
drone6='radio://0/79/2M/E7E7E7E7E6'
drone7='radio://0/80/2M/E7E7E7E7E7'
# URI that are actually used in the program
urit = drone5
urif = drone6
# drone parameters
# software bounds to keep the drone from hitting the net
# limits of form [-x,x,-y,y,-z,z]
lims=[-.7,.7,-.4,.6,.2,1.4]
# time to predict forward with velocity
pred=0
# chagnes the frequency of update commands and of the position logging
freq=20
# tracking drone velocity
tvel=.1
# flying drone velocity
fvel=2
# radius of exclusion for drone pathfinding
min_rad=.15
# distance of the target position to the drone
tar_rad=.3
# angle between the horizon of the target drone and the flying drone
hov_ang=math.pi/4
#data logging required for persue
logt = LogConfig(name='Stabilizer', period_in_ms=1000/freq)
logt.add_variable('stateEstimate.x', 'FP16')
logt.add_variable('stateEstimate.y', 'FP16')
logt.add_variable('stateEstimate.z', 'FP16')
logt.add_variable('stateEstimate.yaw', 'FP16')
logt.add_variable('stateEstimate.vx', 'FP16')
logt.add_variable('stateEstimate.vy', 'FP16')
logt.add_variable('stateEstimate.vz', 'FP16')
logf = LogConfig(name='Stabilizer', period_in_ms=1000/freq)
logf.add_variable('stateEstimate.x', 'FP16')
logf.add_variable('stateEstimate.y', 'FP16')
logf.add_variable('stateEstimate.z', 'FP16')
logf.add_variable('stateEstimate.yaw', 'FP16')
logf.add_variable('stateEstimate.vx', 'FP16')
logf.add_variable('stateEstimate.vy', 'FP16')
logf.add_variable('stateEstimate.vz', 'FP16')
#add other variables to be logged below

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    temp=list(data.values())
    temp.append((logconf.data[4]-temp[4])*freq)
    logconf.data=temp

def simple_log_async_start(scf, logconf):
    logconf.data=[0,0,0,0,0,0,0,0,0]
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

def simple_log_async_stop(scf, logconf):
    logconf.stop()
    logconf.data=[]

def kahanP1(u,v):
    u = u/np.linalg.norm(u)
    v = v/np.linalg.norm(v)
    return 2*math.atan(np.linalg.norm(u-v)/np.linalg.norm(u+v)) 

def oobounds(pos):
    for i in range(3):
        if pos[i]<lims[2*i]:
            pos[i]=lims[2*i]
        elif pos[i]>lims[2*i+1]:
            pos[i]=lims[2*i+1]
    return pos

# Finds next location for pursuer to go. 
# Run next line to call function in main
# pursue(fpc, logf.data, logt.data)
def pursue(pc, fdata, tdata):
    # print(tdata)
    # print(tdata[0:3])
    # print(tdata[3]/180*math.pi)
    # print(tdata[4:7])
    T_pos=np.array(tdata[0:3])
    T_yaw=np.array(tdata[3])/180*math.pi
    T_vel=np.array(tdata[4:7])
    T_yawv=np.array(tdata[7])/180*math.pi
    F_pos=np.array(fdata[0:3])
    # calculates the vector
    diff=F_pos-T_pos
    #calculates the yaw of the tracking drone should be the same as the above yaw within error
    #T_yaw=math.atan2(2*(T_quat[3]*T_quat[0]+T_quat[1]*T_quat[2]),-(1-2*(T_quat[0]**2+T_quat[1]**2)))
    #calculates the final drone position that is above and behind the tracking drone
    offset=tar_rad*np.array([-math.cos(T_yaw)*math.cos(hov_ang),-math.sin(T_yaw)*math.cos(hov_ang),math.sin(hov_ang)])
    offsetvel=tar_rad*np.array([math.sin(T_yaw)*math.cos(hov_ang),-math.cos(T_yaw)*math.cos(hov_ang),0])*T_yawv
    Final_pos=T_pos+offset+(T_vel+offsetvel)*pred
    #calculates the straight path vector between the flying drone and the target position
    diff_F=Final_pos-F_pos
    # finds the vector that has the shortest distance between the tracking drone and the straight line path of the flying drone to its target position
    rad_vec=diff-(np.dot(diff,diff_F)/np.dot(diff_F,diff_F)*diff_F)
    # the minimum distance between the path of the flying drone and the tracking drone
    act_rad = np.linalg.norm(rad_vec)
    #current radius from the tracking drone
    cur_rad=np.linalg.norm(diff)
    if cur_rad<min_rad:
        #if the flying drone is currently too close to the tracking drone it will move away in the shortest path until it can use another pathing method to get around
        print("Too close")
        Tar_pos=T_pos+diff/cur_rad*tar_rad
    elif kahanP1(T_pos-Final_pos,F_pos-Final_pos)<math.asin(min_rad/tar_rad) and np.linalg.norm(diff_F)>math.sqrt(tar_rad**2-min_rad**2):
        #if the path goes to close to the tracking drone it creates a path that goes around the drone. not the shortest path but will go around the tracking drone in the shortest direction
        print("Intersecting")
        Tar_pos=T_pos+rad_vec*(act_rad+tar_rad)/act_rad
    else:
        # if there is no collison issue the drone will go straight to the target point
        Tar_pos=Final_pos
    gotoLoc(pc,Tar_pos, T_yaw, fvel)

# Function to go to location, used in pursuer and tracker
def gotoLoc(pc,pos,yaw,v):
    pos = oobounds(pos)
    pc.go_to(pos[0],pos[1],pos[2],yaw,v)

def goToHome():
    gotoLoc(tpc,[0,0,0.1],0,fvel)
    pursue(fpc, logf.data, logt.data)
    time.sleep(2)


    
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    #ensures that both drones are connected and are able to be flown
    with(SyncCrazyflie(urif, cf=Crazyflie(rw_cache='./cachef')) as fscf,
    SyncCrazyflie(urit, cf=Crazyflie(rw_cache='./cachet')) as tscf,
    #PositionHlCommander(tscf, controller=PositionHlCommander.CONTROLLER_PID) as tpc,
    PositionHlCommander(fscf, controller=PositionHlCommander.CONTROLLER_PID) as fpc):
        # starts the logging
        simple_log_async_start(tscf, logt)
        simple_log_async_start(fscf, logf)
        # required to give the logging time to initialize
        time.sleep(1)
        # TODO: Change below code to FSM for diifferent conditions to show off features of our code
        # See nextsteps.txt for more info
        
               
        #Current states:
        #Purerot
        #backandforthy
        #backandforthx
        #upanddown
        #kahandemonstration
        #boundarydemonstration       
        
        # pureRot
        t = 0
        ti = time.time()
        w = 0.1
        while t<5:
            t=time.time()-ti
            gotoLoc(tpc,[0,0,0.1],t*w,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
            
        # home
        goToHome()
        
        # backAndForthY
        t = 0
        ti = time.time()
        w = 0.1
        while t<1:
            t=time.time()-ti
            gotoLoc(tpc,[0,0.5,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0,-0.5,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        t = 0
        ti = time.time()
        w = 0.1
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0,0.5,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0,-0.5,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
            
        # home
        goToHome()    
        
        # backAndForthX
        t = 0
        ti = time.time()
        w = 0.1
        while t<1:
            t=time.time()-ti
            gotoLoc(tpc,[0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[-0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        t = 0
        ti = time.time()
        w = 0.1
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[-0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        
        #Home
        goToHome()
        
        # upAndDown
        t = 0
        ti = time.time()
        w = 0.1
        while t<1:
            t=time.time()-ti
            gotoLoc(tpc,[0,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        t = 0
        ti = time.time()
        w = 0.1
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        while t<2:
            t=time.time()-ti
            gotoLoc(tpc,[-0.5,0,0.1],0,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
        
        # kahandemonstration
        
        
        #boundarydemonstration
        
        #weird ass function
        while t<20:
            t=time.time()-ti
            gotoLoc(tpc,[.4*math.sin(w*t),.4*math.cos(w*t),1],t*w,fvel)
            pursue(fpc, logf.data, logt.data)
            time.sleep(1/freq)
