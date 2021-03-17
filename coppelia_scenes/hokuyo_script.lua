function sysCall_init() 
    if simROS then
        sim.addLog(sim.verbosity_scriptinfos,"ROS interface was found.")
        
        -- local sysTime=sim.getSystemTimeInMs(-1)
        local hokuyoTopicName='/p3dx/laser/scan'
        
        -- Prepare the hokuyo publisher:
        hokuyoPub=simROS.advertise(hokuyoTopicName,'sensor_msgs/LaserScan')
        
        -- Now we start the client application:
        -- result=sim.launchExecutable('Pioneer_p3dx',hokuyoTopicName,0)
    
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Cannot run.")
    end
    laserHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01_laser")
    jointHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01_joint")
    modelRef=sim.getObjectHandle("Hokuyo_ref")
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)
    communicationTube=sim.tubeOpen(0,objName..'_HOKUYO',1)
    scanningAngle=240*math.pi/180
    rotationalVelocity=2*math.pi*10
    stepSize=2*math.pi/1024
    pts=684-- for one scan pass
    points={}
    ranges={}
    segments={}
    segmentsFreshnessID={}
    
    for i=1,pts*3,1 do
        table.insert(points,0)
    end
    for i=1,pts do
        table.insert(ranges,0)
    end
    for i=1,pts*7,1 do
        table.insert(segments,0)
    end
    for i=1,pts,1 do
        table.insert(segmentsFreshnessID,0)
    end

    black={0,0,0}
    red={1,0,0}
    lines100=sim.addDrawingObject(sim.drawing_lines,1,0,-1,1000,black,black,black,red)
    lines75=sim.addDrawingObject(sim.drawing_lines+sim.drawing_25percenttransparency,1,0,-1,1000,black,black,black,red)
    lines50=sim.addDrawingObject(sim.drawing_lines+sim.drawing_50percenttransparency,1,0,-1,1000,black,black,black,red)
    lines25=sim.addDrawingObject(sim.drawing_lines+sim.drawing_50percenttransparency+sim.drawing_25percenttransparency,1,0,-1,1000,black,black,black,red)

    points100=sim.addDrawingObject(sim.drawing_points,4,0,-1,1000,black,black,black,red)
    points75=sim.addDrawingObject(sim.drawing_points+sim.drawing_25percenttransparency,4,0,-1,1000,black,black,black,red)
    points50=sim.addDrawingObject(sim.drawing_points+sim.drawing_50percenttransparency,4,0,-1,1000,black,black,black,red)
    points25=sim.addDrawingObject(sim.drawing_points+sim.drawing_50percenttransparency+sim.drawing_25percenttransparency,4,0,-1,1000,black,black,black,red)

    previousDAngleRest=0
    currentFreshnessID=1
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines100)
    sim.removeDrawingObject(lines75)
    sim.removeDrawingObject(lines50)
    sim.removeDrawingObject(lines25)
    sim.removeDrawingObject(points100)
    sim.removeDrawingObject(points75)
    sim.removeDrawingObject(points50)
    sim.removeDrawingObject(points25)
end 

function sysCall_sensing() 
    showLaserPoints=sim.getScriptSimulationParameter(sim.handle_self,'showLaserPoints')
    showLaserSegments=sim.getScriptSimulationParameter(sim.handle_self,'showLaserSegments')
    fullScanPerSimulationPass=sim.getScriptSimulationParameter(sim.handle_self,'fullScanPerSimulationPass')
    
    dt=sim.getSimulationTimeStep()
    if fullScanPerSimulationPass then
        dAngle=math.pi*2 -- Non-realistic scanning (instantaneous scan)
    else
        dAngle=-previousDAngleRest+dt*rotationalVelocity -- Realistic scanning
    end
    
    angle=0
    jointPos=sim.getJointPosition(jointHandle)
    
    for i=1,#segmentsFreshnessID,1 do
        if (segmentsFreshnessID[i]==currentFreshnessID) then
            segmentsFreshnessID[i]=0
        end
    end
    
    laserOrigin=sim.getObjectPosition(jointHandle,-1)
    modelInverseMatrix=simGetInvertedMatrix(sim.getObjectMatrix(modelRef,-1))
    stamp=simROS.getTime()
    firstAngle = angle
    while angle<dAngle do
        if firstAngle==angle then
            scan_time=simROS.getTime()
        end
        angle=angle+stepSize
        jointPos=jointPos+stepSize
        if (jointPos>math.pi) then
            jointPos=jointPos-2*math.pi
        end
        sim.setJointPosition(jointHandle,jointPos)
        if (jointPos>=-scanningAngle*0.5)and(jointPos<=scanningAngle*0.5) then
            ind=math.floor(0+pts*(jointPos+scanningAngle*0.5)/scanningAngle)
            r,dist,pt=sim.handleProximitySensor(laserHandle) -- pt is relative to the laser ray! (rotating!)
            m=sim.getObjectMatrix(laserHandle,-1)
            if r>0 then
                -- We put the RELATIVE coordinate of that point into the table that we will return:
                ptAbsolute=sim.multiplyVector(m,pt)
                ptRelative=sim.multiplyVector(modelInverseMatrix,ptAbsolute)
                points[3*ind+1]=ptRelative[1]
                points[3*ind+2]=ptRelative[2]
                points[3*ind+3]=ptRelative[3]
                ranges[ind+1]=math.sqrt(ptRelative[1]*ptRelative[1] + ptRelative[2]*ptRelative[2])
                segments[7*ind+7]=1 -- indicates a valid point
            else
                -- If we didn't detect anything, we specify (0,0,0) for the coordinates:
                ptAbsolute=sim.multiplyVector(m,{0,0,6})
                points[3*ind+1]=0
                points[3*ind+2]=0
                points[3*ind+3]=0
                ranges[ind+1]=0
                segments[7*ind+7]=0 -- indicates an invalid point
            end
            segments[7*ind+1]=laserOrigin[1]
            segments[7*ind+2]=laserOrigin[2]
            segments[7*ind+3]=laserOrigin[3]
            segments[7*ind+4]=ptAbsolute[1]
            segments[7*ind+5]=ptAbsolute[2]
            segments[7*ind+6]=ptAbsolute[3]
            segmentsFreshnessID[ind+1]=currentFreshnessID
        end
    end
    
    fr={currentFreshnessID,currentFreshnessID-1,currentFreshnessID-2,currentFreshnessID-3}
    for i=2,4,1 do
        if (fr[i]<1) then
            fr[i]=fr[i]+4
        end
    end
    
    sim.addDrawingObjectItem(lines100,nil)
    sim.addDrawingObjectItem(lines75,nil)
    sim.addDrawingObjectItem(lines50,nil)
    sim.addDrawingObjectItem(lines25,nil)
    
    sim.addDrawingObjectItem(points100,nil)
    sim.addDrawingObjectItem(points75,nil)
    sim.addDrawingObjectItem(points50,nil)
    sim.addDrawingObjectItem(points25,nil)
    
    if (showLaserPoints or showLaserSegments) then
        t={0,0,0,0,0,0}
        for i=0,#segmentsFreshnessID-1,1 do
            t[1]=segments[7*i+4]
            t[2]=segments[7*i+5]
            t[3]=segments[7*i+6]
            t[4]=segments[7*i+1]
            t[5]=segments[7*i+2]
            t[6]=segments[7*i+3]
            if (segmentsFreshnessID[i+1]==fr[1]) then
                if showLaserSegments then
                    sim.addDrawingObjectItem(lines100,t)
                end
                if (showLaserPoints and segments[7*i+7]~=0)then
                    sim.addDrawingObjectItem(points100,t)
                end
            end
            if (segmentsFreshnessID[i+1]==fr[2]) then
                if showLaserSegments then
                    sim.addDrawingObjectItem(lines75,t)
                end
                if (showLaserPoints and segments[7*i+7]~=0)then
                    sim.addDrawingObjectItem(points75,t)
                end
            end
            if (segmentsFreshnessID[i+1]==fr[3]) then
                if showLaserSegments then
                    sim.addDrawingObjectItem(lines50,t)
                end
                if (showLaserPoints and segments[7*i+7]~=0)then
                    sim.addDrawingObjectItem(points50,t)
                end
            end
            if (segmentsFreshnessID[i+1]==fr[4]) then
                if showLaserSegments then
                    sim.addDrawingObjectItem(lines25,t)
                end
                if (showLaserPoints and segments[7*i+7]~=0)then
                    sim.addDrawingObjectItem(points25,t)
                end
            end
        end
    end
    
    previousDAngleRest=angle-dAngle
    currentFreshnessID=currentFreshnessID+1
    if (currentFreshnessID>4) then
        currentFreshnessID=1
    end
    
    -- Now send the data:
    if #points>0 then
        sim.tubeWrite(communicationTube,sim.packFloatTable(points))
    end
    
    -- To read the data from another script, use following instructions (in that other script):
    --
    -- INITIALIZATION PHASE:
    -- laserScannerHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01")
    -- laserScannerObjectName=sim.getObjectName(laserScannerHandle) -- is not necessarily "Hokuyo_URG_04LX_UG01"!!!
    -- communicationTube=sim.tubeOpen(0,laserScannerObjectName..'_HOKUYO',1)
    --
    -- TO READ THE DATA:
    -- data=sim.tubeRead(communicationTube)
    -- if (data) then
    --     laserDetectedPoints=sim.unpackFloatTable(data)
    -- end
    --
    -- The data in laserDetectedPoints will be RELATIVE to the laser scanner base! 
    
end 

function sysCall_actuation()
    -- data=sim.tubeRead(communicationTube)
    if (points) then
        -- laserDetectedPoints=sim.unpackFloatTable(data)
        
        -- print(ranges)
        msg={
            header={
                stamp=stamp,
                frame_id='lms100',
            },
            angle_min = -120 * math.pi/180,
            angle_max = 120 * math.pi/180,
            angle_increment = 2*math.pi/1024, -- 0.25 * math.pi/180,
            time_increment = (1 / 50) / 1081,
            scan_time=scan_time,
            range_min = 0,
            range_max = 6,
            ranges = ranges,
            intensities = {}
        }
        
        simROS.publish(hokuyoPub, msg)
        
    end
end

