#!/usr/bin/env python3

from register_sensor import imu, gnss, uwb
from kinematics_kf import kalman_filter
import numpy as np
import rclpy
import time

def main(args=None):
    rclpy.init(args=args)
    ## IMU objects (name, topic, frame)
    # sbg = imu('mavros-imu', '/mavros/imu/data', 'ENU')
    sbg = imu('sbg', '/imu/data', 'NWU')
    # zed = imu('zed_imu', 'zed/zed_node/imu/data', 'ENU')

    ## GPS objects (name, topic, origin)
    #gps = gnss('xsens-gps', '/xsens/gnss', [])
    # gps = gnss('ublox_gps', '/ublox/fix')  # /lidom/gps/fix   /mavros/global_position/raw/fix
    #gps = gnss('ardusimple-gps', '/ardusimple/gnss')
    gps = gnss('ublox_gps', '/ublox_gps_node/fix')
    ## UWB objects
    uwb_sensor = uwb('uwb', '/uwb/loc')

    num_states = 8
    num_measurements = 6

    ''' Initial states and Input parameters '''
    ''' (x,y,psi,u,v,r,ax,ay) '''
    # initial_state = np.array([65,-29,0.0,0,0,0,0,0])                        # East concrete pillar
    # initial_state = np.array([-82.51,-2.47,0.0,0,0,0,0,0])                # West concrete pillar wrt old rusted tank
    initial_state = np.array([0.0,0.0,0,0,0,0,0,0])            # Wave Basin
    # initial_state = np.array([0, 0, 0, 1, 0, 0, 0, 0])

    initial_covariance = 1e-6*np.identity(num_states)
    initial_covariance[0,0] = initial_covariance[1,1] = 1e-3 # 0.1 for gps
    initial_covariance[2,2] = 5*1e-4

    '''
    ( 0 , 1,  2,  3, 4, 5)
    ( x,  y, psi, r, ax, ay)
    '''
    measurement_noise = np.zeros((num_measurements,num_measurements))
    # measurement_noise[0,0] = 0.03 #0.000001#1e-05   #0.03  # 2.386235916299923e-13
    # measurement_noise[1,1] = 0.03 #0.000001 #1e-05  #0.03 # 7.084323962612105e-13
    # measurement_noise[2,2] = 0.0007951435225428663#7.131028243600414e-05    #3*1e-5 # 0.06
    # measurement_noise[3,3] = 2.605696980476807e-07     #8*1e-7 #1.52e-6#
    # measurement_noise[4,4] = 5.512515053771244e-05#3.178294051157069e-05    #4*1e-6 #0.01872 #0.0002#
    # measurement_noise[5,5] = 4.9083797879270436e-05#1.8546761247394648e-05#2.14477382446546e-05    #4*1e-6 #0.00235 #0.0002#
    measurement_noise[0,0] = 1.92e-4     #0.03(gps) #0.125(uwb)
    measurement_noise[1,1] = 1.92e-4    #0.03(gps) #0.278(uwb)
    measurement_noise[2,2] = 1.28*1e-3     #3*1e-5 # 0.06
    measurement_noise[3,3] = 8*1e-1     #8*1e-7 #1.52e-6#
    measurement_noise[4,4] = 4*1e8     #4*1e-6 #0.01872 #0.0002#
    measurement_noise[5,5] = 4*1e8    #4*1e-6 #0.00235 #0.0002#

    '''
    (0, 1,  2 , 3, 4, 5, 6 , 7 )
    (x, y, psi, u, v, r, ax, ay)
    '''
    process_noise = np.zeros((num_states,num_states))
    process_noise[0,0] = process_noise[1,1] = 1e-2
    process_noise[3,3] = process_noise[4,4] = 3e3           ####  lin. velocities  
    process_noise[6,6] = process_noise[7,7] = 6e6           ####  accelerations
    process_noise[2,2] = 0.06
    process_noise[5,5] = 0.02

    filter_freqency = 10
    ## Kinematic_kf.py -> kalman_filter instance
    kf = kalman_filter('kalman_filter_node', filter_freqency ) # kalman filter instance
    kf.register_sensors([sbg], [gps], [uwb_sensor])
    kf.filtered_state = initial_state
    kf.state_covariance = initial_covariance
    kf.measurement_noise = measurement_noise
    kf.process_noise = process_noise

    while rclpy.ok():
        try:
            rclpy.spin_once(kf)
            kf.filter()
            time.sleep(1.0 / filter_freqency )
        except KeyboardInterrupt:
            kf.destroy_node()

if __name__ == '__main__':
    main()
