function sysCall_init()
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        
        -- local sysTime=sim.getSystemTimeInMs(-1)
        local hokuyoTopicName='/pioneer_2/points1'
        
        -- Prepare the hokuyo publisher:
        hokuyoPub=simROS.advertise(hokuyoTopicName,'sensor_msgs/PointCloud')
        
        -- Now we start the client application:
        result=sim.launchExecutable('Pioneer_p3dx',hokuyoTopicName,0)
    
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
    
    visionSensor1Handle=sim.getObjectHandle("fastHokuyo_sensor1")
    visionSensor2Handle=sim.getObjectHandle("fastHokuyo_sensor2")
    joint1Handle=sim.getObjectHandle("fastHokuyo_joint1")
    joint2Handle=sim.getObjectHandle("fastHokuyo_joint2")
    sensorRef=sim.getObjectHandle("fastHokuyo_ref")
    
    maxScanDistance=sim.getScriptSimulationParameter(sim.handle_self,'maxScanDistance')
    if maxScanDistance>1000 then maxScanDistance=1000 end
    if maxScanDistance<0.1 then maxScanDistance=0.1 end
    sim.setObjectFloatParameter(visionSensor1Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParameter(visionSensor2Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    maxScanDistance_=maxScanDistance*0.9999
    
    scanningAngle=sim.getScriptSimulationParameter(sim.handle_self,'scanAngle')
    if scanningAngle>240 then scanningAngle=240 end
    if scanningAngle<2 then scanningAngle=2 end
    scanningAngle=scanningAngle*math.pi/180
    sim.setObjectFloatParameter(visionSensor1Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)
    sim.setObjectFloatParameter(visionSensor2Handle,sim.visionfloatparam_perspective_angle,scanningAngle/2)

    sim.setJointPosition(joint1Handle,-scanningAngle/4)
    sim.setJointPosition(joint2Handle,scanningAngle/4)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,10000,nil,nil,nil,red)

    if (sim.getInt32Parameter(sim.intparam_program_version)<30004) then
        sim.displayDialog("ERROR","This version of the Hokuyo sensor is only supported from CoppeliaSim V3.0.4 and upwards.&&nMake sure to update your CoppeliaSim.",sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines)
end 

function sysCall_sensing() 
    measuredData={}
    
    if notFirstHere then
        -- We skip the very first reading
        sim.addDrawingObjectItem(lines,nil)
        showLines=sim.getScriptSimulationParameter(sim.handle_self,'showLaserSegments')
        r,t1,u1=sim.readVisionSensor(visionSensor1Handle)
        r,t2,u2=sim.readVisionSensor(visionSensor2Handle)
    
        m1=sim.getObjectMatrix(visionSensor1Handle,-1)
        m01=simGetInvertedMatrix(sim.getObjectMatrix(sensorRef,-1))
        m01=sim.multiplyMatrices(m01,m1)
        m2=sim.getObjectMatrix(visionSensor2Handle,-1)
        m02=simGetInvertedMatrix(sim.getObjectMatrix(sensorRef,-1))
        m02=sim.multiplyMatrices(m02,m2)
                
        if u1 then
            p={0,0,0}
            p=sim.multiplyVector(m1,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u1[2]-1,1 do
                for i=0,u1[1]-1,1 do
                    w=2+4*(j*u1[1]+i)
                    v1=u1[w+1]
                    v2=u1[w+2]
                    v3=u1[w+3]
                    v4=u1[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m01,p)
                        local point={
                            x=p[1],
                            y=p[2],
                            z=p[3],
                        }
                        table.insert(measuredData,point)
                        -- table.insert(measuredData,p[1])
                        -- table.insert(measuredData,p[2])
                        -- table.insert(measuredData,p[3])
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m1,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        if u2 then
            p={0,0,0}
            p=sim.multiplyVector(m2,p)
            t={p[1],p[2],p[3],0,0,0}
            for j=0,u2[2]-1,1 do
                for i=0,u2[1]-1,1 do
                    w=2+4*(j*u2[1]+i)
                    v1=u2[w+1]
                    v2=u2[w+2]
                    v3=u2[w+3]
                    v4=u2[w+4]
                    if (v4<maxScanDistance_) then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m02,p)
                        local point={
                            x=p[1],
                            y=p[2],
                            z=p[3],
                        }
                        table.insert(measuredData,point)
                        -- table.insert(measuredData,p[1])
                        -- table.insert(measuredData,p[2])
                        -- table.insert(measuredData,p[3])
                    end
                    if showLines then
                        p={v1,v2,v3}
                        p=sim.multiplyVector(m2,p)
                        t[4]=p[1]
                        t[5]=p[2]
                        t[6]=p[3]
                        sim.addDrawingObjectItem(lines,t)
                    end
                end
            end
        end
        
        local msg={
            header={
                stamp=simROS.getTime(), 
                frame_id='lms100'
            }, 
            points=measuredData, 
            channels={}
            }
            
        -- print(msg)
        simROS.publish(hokuyoPub, msg)
    
    end
    notFirstHere=true
    
    -- measuredData now contains all the points that are closer than the sensor range
    -- For each point there is the x, y and z coordinate (i.e. 3 number for each point)
    -- Coordinates are expressed relative to the sensor frame.
    -- You can access this data from outside via various mechanisms. The best is to first
    -- pack the data, then to send it as a string. For example:
    --
    -- 
    --
    -- Then in a different location:
    -- data=sim.getStringSignal("measuredDataAtThisTime")
    -- measuredData=sim.unpackFloatTable(data)
    --
    --
    -- Of course you can also send the data via tubes, wireless (sim.tubeOpen, etc., sim.sendData, etc.)
    --
    -- Also, if you send the data via string signals, if you you cannot read the data in each simulation
    -- step, then always append the data to an already existing signal data, e.g.
    --
    -- 
    -- data=sim.packFloatTable(measuredData)
    -- existingData=sim.getStringSignal("measuredDataAtThisTime")
    -- if existingData then
    --     data=existingData..data
    -- end
    -- sim.setStringSignal("measuredDataAtThisTime",data)
end 

function sysCall_actuation()
    local laser_scan = sim.getStringSignal("measuredDataAtThisTime")
        
    if laser_scan then
        
    end
end