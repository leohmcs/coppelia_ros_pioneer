function sysCall_init() 
    modelBase=sim.getObjectAssociatedWithScript(sim.handle_self)
    massObject=sim.getObjectHandle('Accelerometer_mass')
    sensor=sim.getObjectHandle('Accelerometer_forceSensor')
    result,mass=sim.getObjectFloatParameter(massObject,sim.shapefloatparam_mass)
    ui=simGetUIHandle('Accelerometer_UI')
    simSetUIButtonLabel(ui,0,sim.getObjectName(modelBase))
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1)
end
-- Check the end of the script for some explanations!


function sysCall_cleanup() 
 
end 

function sysCall_sensing() 
    result,force=sim.readForceSensor(sensor)
    if (result>0) then
        accel={force[1]/mass,force[2]/mass,force[3]/mass}
        sim.tubeWrite(accelCommunicationTube,sim.packFloatTable(accel))
        simSetUIButtonLabel(ui,3,string.format("X-Accel: %.4f",accel[1]))
        simSetUIButtonLabel(ui,4,string.format("Y-Accel: %.4f",accel[2]))
        simSetUIButtonLabel(ui,5,string.format("Z-Accel: %.4f",accel[3]))
    else
        simSetUIButtonLabel(ui,3,"X-Accel: -")
        simSetUIButtonLabel(ui,4,"Y-Accel: -")
        simSetUIButtonLabel(ui,5,"Z-Accel: -")
    end
    
    -- To read data from this accelerometer in another script, use following code:
    --
    -- accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    -- data=sim.tubeRead(accelCommunicationTube)
    -- if (data) then
    --     acceleration=sim.unpackFloatTable(data)
    -- end
    --
    -- If the script in which you read the acceleration has a different suffix than the accelerometer suffix,
    -- then you will have to slightly adjust the code, e.g.:
    -- accelCommunicationTube=sim.tubeOpen(0,'accelerometerData#') -- if the accelerometer script has no suffix
    -- or
    -- accelCommunicationTube=sim.tubeOpen(0,'accelerometerData#0') -- if the accelerometer script has a suffix 0
    -- or
    -- accelCommunicationTube=sim.tubeOpen(0,'accelerometerData#1') -- if the accelerometer script has a suffix 1
    -- etc.
    --
    --
    -- You can of course also use global variables (not elegant and not scalable), e.g.:
    -- In the accelerometer script:
    -- sim.setFloatSignal('accelerometerX',accel[1])
    -- sim.setFloatSignal('accelerometerY',accel[2])
    -- sim.setFloatSignal('accelerometerZ',accel[3])
    --
    -- And in the script that needs the data:
    -- xAccel=sim.getFloatSignal('accelerometerX')
    -- yAccel=sim.getFloatSignal('accelerometerY')
    -- zAccel=sim.getFloatSignal('accelerometerZ')
    --
    -- In addition to that, there are many other ways to have 2 scripts exchange data. Check the documentation for more details
end 
