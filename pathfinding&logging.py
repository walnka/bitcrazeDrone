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
urit = drone7
urif = drone6
# drone parameters
# software bounds to keep the drone from hitting the net
# limits of form [-x,x,-y,y,-z,z]
lims=[-1,.7,-.6,.7,.3,1.4] 
# time to predict forward with velocity
pred=.5
# chagnes the frequency of update commands and of the position logging
freq=40
# tracking drone velocity
tvel=.2
# flying drone velocity
fvel=1
# radius of exclusion for drone pathfinding
min_rad=.2
# distance of the target position to the drone
tar_rad=.3
# angle between the horizon of the target drone and the flying drone
hov_ang=math.pi/12
# data logging required for persue
logt = LogConfig(name='Stabilizer', period_in_ms=1000/freq)
logt.add_variable('stateEstimateZ.x', 'int16_t')
logt.add_variable('stateEstimateZ.y', 'int16_t')
logt.add_variable('stateEstimateZ.z', 'int16_t')
logt.add_variable('stateEstimate.yaw', 'FP16')
logt.add_variable('stateEstimateZ.vx', 'int16_t')
logt.add_variable('stateEstimateZ.vy', 'int16_t')
logt.add_variable('stateEstimateZ.vz', 'int16_t')
logt.add_variable('stateEstimateZ.rateYaw', 'int16_t')
# uncomment to use accelerometer data, leave commented to calculate acceleration using the above velocity
# logt.add_variable('stateEstimateZ.ax', 'int16_t')
# logt.add_variable('stateEstimateZ.ay', 'int16_t')
# logt.add_variable('stateEstimateZ.az', 'int16_t')
logf = LogConfig(name='Stabilizer', period_in_ms=1000/freq)
logf.add_variable('stateEstimateZ.x', 'int16_t')
logf.add_variable('stateEstimateZ.y', 'int16_t')
logf.add_variable('stateEstimateZ.z', 'int16_t')
# logf.add_variable('stateEstimate.yaw', 'FP16')
# logf.add_variable('stateEstimateZ.vx', 'int16_t')
# logf.add_variable('stateEstimateZ.vy', 'int16_t')
# logf.add_variable('stateEstimateZ.vz', 'int16_t')
# logf.add_variable('stateEstimateZ.rateYaw', 'int16_t')
# logf.add_variable('stateEstimateZ.ax', 'int16_t')
# logf.add_variable('stateEstimateZ.ay', 'int16_t')
# logf.add_variable('stateEstimateZ.az', 'int16_t')
#add other variables to be logged below

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    temp=list(np.array(list(data.values()))/1000)
    if len(temp)>4:
        temp[3]=temp[3]*1000/180*math.pi
        if len(temp)>=9: # calculates acceleration from onboard accelerometer
            temp[10]=temp[10]-9.81
            temp.append((temp[7]-logconf.data[7])*freq)
        else: # caclulates acceleration from velocity
            for i in range(4):
                temp.append((temp[4+i]-logconf.data[4+i])*freq)
    else: # calculates acceleration and velocity form just position data
        for i in range(8):
            temp.append((temp[i]-logconf.data[i])*freq)
    logconf.data=list(temp)

def simple_log_async_start(scf, logconf):
    logconf.data=[0,0,0,0,0,0,0,0,0,0,0,0]
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
    T_yaw=np.array(tdata[3])
    offset=tar_rad*np.array([-math.cos(T_yaw)*math.cos(hov_ang),-math.sin(T_yaw)*math.cos(hov_ang),math.sin(hov_ang)])
    # calculates the final target position of the dorne
    Final_pos=T_pos+offset
    # if we are recording the velocity data it will use this to predict a final position of the drone
    if len(tdata)>=8:
        T_vel=np.array(tdata[4:7])
        T_yawv=np.array(tdata[7])
        offsetvel=tar_rad*np.array([math.sin(T_yaw)*math.cos(hov_ang),-math.cos(T_yaw)*math.cos(hov_ang),0])*T_yawv
        Final_pos=Final_pos+(T_vel+offsetvel)*pred
        # if we are recording the acceleration data it will use this to predict the final position of the drone
        if len(tdata)>=12:
            T_acc=np.array(tdata[8:11])
            T_yawa=np.array(tdata[11])
        else:
            T_acc=np.array([0,0,0])
            T_yawa=0
        offsetacc=tar_rad*(np.array([math.sin(T_yaw)*math.cos(hov_ang),-math.cos(T_yaw)*math.cos(hov_ang),0])*T_yawa+np.array([math.cos(T_yaw)*math.cos(hov_ang),math.sin(T_yaw)*math.cos(hov_ang),0])*T_yawv**2)
        Final_pos=Final_pos+(T_acc+offsetacc)/2*pred**2
    F_pos=np.array(fdata[0:3])
    # calculates the vector between the 2 drones
    diff=F_pos-T_pos
    #calculates the straight path vector between the flying drone and the target position
    diff_F=Final_pos-F_pos
    # finds the vector that has the shortest distance between the tracking drone and the straight line path of the flying drone to its target position
    rad_vec=diff-(np.dot(diff,diff_F)/np.dot(diff_F,diff_F)*diff_F)
    # the minimum distance between the path of the flying drone and the tracking drone
    act_rad = np.linalg.norm(rad_vec)
    # current radius from the tracking drone
    cur_rad=np.linalg.norm(diff)
    if cur_rad<min_rad:
        #if the flying drone is currently too close to the tracking drone it will move away in the shortest path until it can use another pathing method to get around
        # print("Too close")
        Tar_pos=T_pos+diff/cur_rad*tar_rad
    elif kahanP1(T_pos-Final_pos,F_pos-Final_pos)<math.asin(min_rad/tar_rad) and np.linalg.norm(diff_F)>math.sqrt(tar_rad**2-min_rad**2): #and cur_rad<tar_rad:
        # if the path goes to close to the tracking drone it creates a path that goes around the drone. not the shortest path but will go around the tracking drone in the shortest direction
        # print("Intersecting")
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
    gotoLoc(tpc,[0,0,0.1],0,tvel)
    pursue(fpc, logf.data, logt.data)
    time.sleep(2)


    
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    # ensures that both drones are connected and are able to be flown
    with SyncCrazyflie(urif, cf=Crazyflie(rw_cache='./cachef')) as fscf:
        with SyncCrazyflie(urit, cf=Crazyflie(rw_cache='./cachet')) as tscf:
          with PositionHlCommander(tscf, controller=PositionHlCommander.CONTROLLER_PID) as tpc:
            with PositionHlCommander(fscf, controller=PositionHlCommander.CONTROLLER_PID) as fpc:
                # starts the logging
                simple_log_async_start(tscf, logt)
                simple_log_async_start(fscf, logf)
                # required to give the logging time to initialize
                time.sleep(2)
                # TODO: Change below code to FSM for diifferent conditions to show off features of our code
                # See nextsteps.txt for more info
                
                       
                # Current states:
                # Purerot
                # backandforthy
                # backandforthx
                # upanddown
                # kahandemonstration
                # boundarydemonstration       
                

                # too close test
                t = 0
                ti = time.time()
                while t<3:
                    t=time.time()-ti
                    gotoLoc(tpc,[0,0,0.6],0,fvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                gotoLoc(fpc,[0,.5,0.6],0,fvel)
                time.sleep(3)
                gotoLoc(fpc,[0,0.2,0.6],0,fvel)
                time.sleep(3)
                t = 0
                ti = time.time()
                while t<5:
                    t=time.time()-ti
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)

                # pureRot
                w=2*math.pi/10
                t = 0
                ti = time.time()
                while t<10:
                    t=time.time()-ti
                    gotoLoc(tpc,[0,0,0.5-t/1000],2*t*w,tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)

                # moving in a circle with rotation
                ti = time.time()
                t=0
                while t<20:
                    t=time.time()-ti
                    gotoLoc(tpc,[.1*math.sin(w*t),.1*math.cos(w*t),.5],2*t*w,tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                #after this it goes crazy
                
                # backAndForthY
                t = 0
                ti = time.time()
                while t<8:
                    t=time.time()-ti
                    if math.floor(t) % 4 <2:
                        gotoLoc(tpc,[0,.5,0.1],0,tvel)
                    else:
                        gotoLoc(tpc,[0,-0.5,0.1],0,tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                # home
                # goToHome()    
                
                # backAndForthX
                t = 0
                ti = time.time()
                while t<16:
                    t=time.time()-ti
                    if math.floor(t) % 4 <2:
                        gotoLoc(tpc,[0.5,0,0.1],0,tvel)
                    else:
                        gotoLoc(tpc,[-0.5,0,0.1],0,tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                
                # Home
                # goToHome()
                
                # upAndDown
                t = 0
                ti = time.time()
                while t<16:
                    t=time.time()-ti
                    if math.floor(t) % 4 <2:
                        gotoLoc(tpc,[0,0,1.2],0,tvel)
                    else:
                        gotoLoc(tpc,[0,0,0.3],0,tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                
                # kahandemonstration
                goToHome() #going to home position
                t = 0
                ti = time.time()
                turn_time=0.8
                pos_it= (math.sin(hov_ang) * tar_rad)/turn_time # height of f-drone above t-drone in turn_time steps
                while t < turn_time: #turning and going upp - want to tweak this to be faster than f-drone reaction time
                    t =time.time() - ti
                    gotoLoc(tpc, [0, 0, 0.1+ t*pos_it], t*math.pi /turn_time, tvel) #possibly tweak tvel to get higher speed
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                z_pos = t*pos_it
                t = 0
                ti = time.time()
                atk_time= 0.5
                while t< atk_time: #charging directly towards f-drone, assuming almost same height
                    t =time.time() - ti
                    gotoLoc(fpc, [t* tar_rad/atk_time, 0, z_pos], math.pi, tvel) #holds the old position in height and yaw, and charges the drone to trigger kahan- avoidance
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                while t< 15: #gives time for the drone to do kahan, and stays in place some time after, change to lowert amount if new demo after
                    t =time.time() - ti
                    gotoLoc(fpc, [tar_rad, 0, z_pos], math.pi, tvel)
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                # boundarydemonstration
                
                t = 0
                ti = time.time()
                
                while t<2:
                    t =time.time() - ti
                    gotoLoc(fpc, [lims[1]-min_rad/2, 0, (lims[5]-lims[4])/2], 0, tvel)  #stays close to the edge-limit in x-dir, and in the middle of z lims
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                t = 0
                ti = time.time()
                turn_time = 3
                while t<turn_time*2: #rotates two times
                    t =time.time() - ti
                    gotoLoc(fpc, [lims[1]-min_rad/2, 0, lims[5]/2], t*math.pi*2 /turn_time, tvel) #rotates completely to push f-drone into the wall,
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                
                t = 0
                ti = time.time()
                while t<3: #go back to middle
                    t =time.time() - ti
                    gotoLoc(fpc, [0, 0, lims[5]/2], 0, tvel) #rotates completely to push f-drone into the wall,
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                while t<8: #go back to middle
                    t =time.time() - ti
                    goToHome()
                    pursue(fpc, logf.data, logt.data)
                    time.sleep(1/freq)
                    
                #want to land now
    
    
    
    
