function sysCall_init()
    robotHandle=sim.getObjectAssociatedWithScript(sim.handle_self)

    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")

    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        
        -- local sysTime=sim.getSystemTimeInMs(-1)
        local robotName='_p3dx' 
        local leftMotorTopicName='leftMotorSpeed'..robotName -- we add a random component so that we can have several instances of this robot running
        local rightMotorTopicName='rightMotorSpeed'..robotName -- we add a random component so that we can have several instances of this robot running
        local simulationTimeTopicName='simTime'..robotName -- we add a random component so that we can have several instances of this robot running
        
        -- Prepare the motor speed subscribers:
        simTimePub=simROS.advertise('/'..simulationTimeTopicName,'std_msgs/Float32')
        leftMotorSub=simROS.subscribe('/'..leftMotorTopicName,'std_msgs/Float32','setLeftMotorVelocity_cb')
        rightMotorSub=simROS.subscribe('/'..rightMotorTopicName,'std_msgs/Float32','setRightMotorVelocity_cb')
        
        -- Now we start the client application:
        -- result=sim.launchExecutable('Pioneer_p3dx',leftMotorTopicName.." "..rightMotorTopicName.." "..sensorTopicName.." "..simulationTimeTopicName,0)
    
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    
    end


end



function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(motorLeft, msg.data)
    print("Left: "..msg.data.." at "..simROS.getTime())
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(motorRight, msg.data)
    print("Right: "..msg.data.." at "..simROS.getTime())
end


function sysCall_cleanup() 
    
end 
