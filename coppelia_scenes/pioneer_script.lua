function sysCall_init()
    previousLeftVel = 0.0
    previousRightvel=0.0
    
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    
    w_radius = 0.0975
    w_base = 0.1655 * 2
    
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        
        -- local sysTime=sim.getSystemTimeInMs(-1)
        local leftMotorTopicName='pioneer/left_wheel_velocity'
        local rightMotorTopicName='pioneer/right_wheel_velocity'
        -- local simulationTimeTopicName='simTime'..robotName -- we add a random component so that we can have several instances of this robot running
        
        -- Prepare the motor speed subscribers:
        -- simTimePub=simROS.advertise('/'..simulationTimeTopicName,'std_msgs/Float32')
        leftMotorSub=simROS.subscribe('/'..leftMotorTopicName,'std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/'..rightMotorTopicName,'std_msgs/Float32','setRightMotorVelocity_cb')
        
        publisher_joints_right = simROS.advertise('/pioneer/right_wheel_joint_state','sensor_msgs/JointState')
        publisher_joints_left = simROS.advertise('/pioneer/left_wheel_joint_state','sensor_msgs/JointState')
        publishers_motors = {publisher_joints_right, publisher_joints_left}

        -- Prepare odom
        -- publisher_odom = simROS.advertise('/coppelia_odom', 'nav_msgs/Odometry')
        last_time = sim.getSimulationTime()
        robot_x = 0.0
        robot_y = 0.0
        robot_th = 0.0
        current_time = last_time
        
        -- Prepare GT pose
        publisher_gt_relative_pose = simROS.advertise('/gt_relative_pose', 'geometry_msgs/Pose')
        publisher_gt_pose = simROS.advertise('/gt_pose', 'geometry_msgs/Pose')
        initial_pose = sim.getObjectPosition(robotHandle,-1)
        
        -- Now we start the client application:
        -- result=sim.launchExecutable('Pioneer_p3dx',leftMotorTopicName.." "..rightMotorTopicName.." "..sensorTopicName.." "..simulationTimeTopicName,0)
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    
    end

end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    if msg.data ~= previousLeftVel then
        sim.setJointTargetVelocity(motorLeft, msg.data)
        print("Left: "..msg.data.." at "..simROS.getTime())
        previousLeftVel = msg.data
    end
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    if msg.data ~= previousRightVel then
        sim.setJointTargetVelocity(motorRight, msg.data)
        print("Right: "..msg.data.." at "..simROS.getTime())
        previousRightVel = msg.data
    end
end

function euler_to_quaternion(roll, pitch, yaw)
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return {qx, qy, qz, qw}
end

function publish_odometry()
    -- estimate odom
    current_time = sim.getSimulationTime()
    dt = current_time - last_time
    
    rightVel = sim.getJointTargetVelocity(motorRight, -1)
    leftVel = sim.getJointTargetVelocity(motorLeft, -1)
    
    v = w_radius * (rightVel + leftVel) / 2.0
    w = w_radius * (rightVel - leftVel) / w_base
    
    vx = v * math.cos(robot_th)
    vy = v * math.sin(robot_th)

    dx = vx * dt
    dy = vy * dt
    dth = w * dt

    robot_x = robot_x + dx
    robot_y = robot_y + dy
    robot_th = robot_th + dth
    
    -- estimate quaternion from the odometry
    quat_rot = euler_to_quaternion(0, 0, robot_th)
    
    -- get the quaternion from the sim with perfect position
    -- quat_rot = sim.getObjectQuaternion(robotHandle,-1)
    
    odom = {
        header= {
            stamp= simROS.getTime(),
            frame_id= 'odom'
        },
        child_frame_id= 'wheel_odom',
        pose= {
            pose= {
                position= {
                    x=robot_x,
                    y=robot_y,
                    z=0
                },
                orientation= {
                    x=quat_rot[1],
                    y=quat_rot[2],
                    z=quat_rot[3],
                    w=quat_rot[4]
                }
            }
        },
        twist= {
            twist= {
                linear= {
                    x=v,
                    y=0,
                    z=0
                },
                angular= {
                    x=0,
                    y=0,
                    z=w
                }
            }
        }
    }
    simROS.publish(publisher_odom, odom)
    
    simROS.sendTransform({
        header={
           stamp=simROS.getTime(), 
            frame_id='odom'}, 
            child_frame_id='base_link', 
            transform={
                translation={x=robot_x, y=robot_y, z=0}, 
                rotation={x=quat_rot[1], y=quat_rot[2], z=quat_rot[3], w=quat_rot[4]}
            }
        }
    )
    
    last_time = current_time
end

function sysCall_sensing()
end

function sysCall_actuation()
    -- publish GT odom/pose
    if (initial_pose) then
        p=sim.getObjectPosition(robotHandle,-1)
        o=sim.getObjectQuaternion(robotHandle,-1)
        rel_pose = {}
        rel_pose['position'] = {x = p[1]-initial_pose[1], y = p[2]- initial_pose[2], z = p[3]- initial_pose[3]}
        rel_pose['orientation']= {x = o[1], y = o[2], z = o[3], w = o[4]}
        simROS.publish(publisher_gt_relative_pose, rel_pose)
        
        pose = {}
        pose['position'] = {x = p[1], y = p[2], z = p[3]}
        pose['orientation']= {x = o[1], y = o[2], z = o[3], w = o[4]}
        -- simROS.publish(publisher_gt_pose, pose)
    end
    
    -- publish joint states of the motor frames
    motors_handlers = {motorRight, motorLeft}
    motor_frames = {"p3dx_right_wheel", "p3dx_left_wheel"}
    for i=1,2,1 do  
        joint_state_msg = {}
        joint_state_msg["header"] = {stamp=sim.getSimulationTime(), frame_id=motor_frames[i]}
        joint_state_msg["name"] = {motor_frames[i]}
        joint_pos = sim.getJointPosition(motors_handlers[i])
        joint_state_msg["position"] = {joint_pos}
        joint_vel = sim.getJointTargetVelocity(motors_handlers[i])
        joint_state_msg["velocity"] = {joint_vel}
        joint_eff = sim.getJointForce(motors_handlers[i])
        joint_state_msg["effort"] = {joint_eff}
        simROS.publish(publishers_motors[i], joint_state_msg)
    end
    
    -- publish_odometry()
end

function sysCall_cleanup() 
    
end 

