function sysCall_init() 
    modelBase=sim.getObjectAssociatedWithScript(sim.handle_self)

    ref=sim.getObjectHandle('GyroSensor_reference')

    ui=simGetUIHandle('GyroSensor_UI')

    simSetUIButtonLabel(ui,0,sim.getObjectName(modelBase))

    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1)

    oldTransformationMatrix=sim.getObjectMatrix(ref,-1)

    lastTime=sim.getSimulationTime()

end
-- Check the end of the script for some explanations!





function sysCall_cleanup() 
 
end 


function sysCall_sensing() 
    local transformationMatrix=sim.getObjectMatrix(ref,-1)

    local oldInverse=simGetInvertedMatrix(oldTransformationMatrix)

    local m=sim.multiplyMatrices(oldInverse,transformationMatrix)

    local euler=sim.getEulerAnglesFromMatrix(m)

    local currentTime=sim.getSimulationTime()

    local gyroData={0,0,0}

    local dt=currentTime-lastTime

    if (dt~=0) then

        gyroData[1]=euler[1]/dt

        gyroData[2]=euler[2]/dt

        gyroData[3]=euler[3]/dt

    end

    sim.tubeWrite(gyroCommunicationTube,sim.packFloatTable(gyroData))

    simSetUIButtonLabel(ui,3,string.format("X-Gyro: %.4f",gyroData[1]))

    simSetUIButtonLabel(ui,4,string.format("Y-Gyro: %.4f",gyroData[2]))

    simSetUIButtonLabel(ui,5,string.format("Z-Gyro: %.4f",gyroData[3]))

    oldTransformationMatrix=sim.copyMatrix(transformationMatrix)

    lastTime=currentTime

    

    

    -- To read data from this gyro sensor in another script, use following code:

    --

    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase

    -- data=sim.tubeRead(gyroCommunicationTube)

    -- if (data) then

    --     angularVariations=sim.unpackFloatTable(data)

    -- end

    --

    -- If the script in which you read the gyro sensor has a different suffix than the gyro suffix,

    -- then you will have to slightly adjust the code, e.g.:

    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#') -- if the gyro script has no suffix

    -- or

    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#0') -- if the gyro script has a suffix 0

    -- or

    -- gyroCommunicationTube=sim.tubeOpen(0,'gyroData#1') -- if the gyro script has a suffix 1

    -- etc.

    --

    --

    -- You can of course also use global variables (not elegant and not scalable), e.g.:

    -- In the gyro script:

    -- sim.setFloatSignal('gyroX',angularVariation[1])

    -- sim.setFloatSignal('gyroY',angularVariation[2])

    -- sim.setFloatSignal('gyroZ',angularVariation[3])

    --

    -- And in the script that needs the data:

    -- angularVariationX=sim.getFloatSignal('gyroX')

    -- angularVariationY=sim.getFloatSignal('gyroY')

    -- angularVariationZ=sim.getFloatSignal('gyroZ')

    --

    -- In addition to that, there are many other ways to have 2 scripts exchange data. Check the documentation for more details

end 

