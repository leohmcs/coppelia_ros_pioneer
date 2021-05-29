function sysCall_init()
    -- do some initialization here
    
    IMU = sim.getObjectHandle('IMU')
    
    -- Gyro
    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1)
    -- Accel
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1)
    
    -- ROS
    pub_imu = simROS.advertise('/imu/data','sensor_msgs/Imu')    
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()    
  
    -- Gyro
    data=sim.tubeRead(gyroCommunicationTube)
    if (data) then
        angularVariations=sim.unpackFloatTable(data)
    end
    
    -- Accel
    data2=sim.tubeRead(accelCommunicationTube)
    if (data2) then
        acceleration=sim.unpackFloatTable(data2)
    end
    
    -- Orientation
    quaternion = sim.getObjectQuaternion(IMU,-1)
    
    
    -- ROS
    local msg = {}
    msg['header'] = {stamp = simROS.getTime(), frame_id = 'imu_link'}
    msg['angular_velocity'] = {x = angularVariations[1], y = angularVariations[2], z = angularVariations[3]}
    msg['angular_velocity_covariance'] = {3.8020e-06, -8.0378e-08, 3.7413e-09, -8.0378e-08, 1.3879e-04, -3.1292e-06, 3.7413e-09, -3.1292e-06, 1.0957e-05}
    msg['linear_acceleration'] = {x = -acceleration[1], y = -acceleration[2], z = -acceleration[3]} -- Need to be inverted to work properly [Adriano]
    msg['linear_acceleration_covariance'] = {0.0031, 9.0655e-07, 6.4202e-07, 9.0655e-07, 1.9329e-05, -3.6999e-05, 6.4202e-07, -3.6999e-05, 1.2417e-04}
    msg['orientation'] = {x = quaternion[1], y = quaternion[2], z = quaternion[3], w = quaternion[4]}
    msg['orientation_covariance'] = {6.2825e-06, -6.4508e-09, 2.3930e-08, -6.4508e-09, 2.4737e-08, 3.8177e-10, 2.3930e-08, 3.8177e-10, 5.2730e-07}
    
       
    simROS.publish(pub_imu, msg) 
end

function sysCall_cleanup()
    simROS.shutdownPublisher(pub_imu)
end

-- See the user manual or the available code snippets for additional callback functions and details

