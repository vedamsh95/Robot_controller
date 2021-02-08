----------------------------------------------------------------------------------
------------------------Functions for Visualization-------------------------------
----------------------------------------------------------------------------------
--put the Values in the Editfields in realtime
publishValues=function()
    if (simUI.getRadiobuttonValue(ui[1],2007)==1) then
        for i=1,6 do
            jp[i]=sim.getJointPosition(jh[i])
            simUI.setEditValue(ui[1],1999+i,tostring(math.round(math.deg(jp[i]),3)))
        end
    else
        for i=1,6 do
            jp[i]=sim.getJointPosition(jh[i])
            simUI.setEditValue(ui[1],1999+i,tostring(math.round(jp[i],3)))
        end
    end
end

--changes the Values in the Editfields
edit_refresh=function()
    local tip_pos = sim.getObjectPosition(tip,-1)
    local tip_ori = sim.getObjectOrientation(tip, -1)
    sim.setObjectPosition(ikTarget,-1,tip_pos)
    sim.setObjectOrientation(ikTarget,-1,tip_ori)
    sim.setObjectPosition(target1,-1,tip_pos)
    sim.setObjectOrientation(target1,-1,tip_ori)
    local new_pos = sim.getObjectPosition(tip,-1)
    local new_ori = get_orientation(tip, -1)
    --local new_ori = sim.getObjectOrientation(tip,-1)
    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui[1],999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui[1],999+i,tostring(math.round(math.deg(new_ori[i-3]),3)))
        end
    end
    if (simUI.getRadiobuttonValue(ui[1],2007)==1) then
        for i=1,6 do
            jp[i]=sim.getJointPosition(jh[i])
            simUI.setEditValue(ui[1],1999+i,tostring(math.round(math.deg(jp[i]),3)))
        end
    else
        for i=1,6 do
            jp[i]=sim.getJointPosition(jh[i])
            simUI.setEditValue(ui[1],1999+i,tostring(math.round(jp[i],3)))
        end
    end
end

--visulaize the endpoint
setConfigEndPoint=function(c)
    local sc = {}
    for i=1,#joints do
        sc[i] = sim.getJointPosition(jh[i])
    end
    for i=1,#joints do
        sim.setJointPosition(jh[i], c[i])
    end
    local tip_pos = sim.getObjectPosition(tip,-1)
    local tip_ori = sim.getObjectOrientation(tip, -1)
    sim.setObjectPosition(ikTarget,-1,tip_pos)
    sim.setObjectOrientation(ikTarget,-1,tip_ori)
    sim.setObjectPosition(target1,-1,tip_pos)
    sim.setObjectOrientation(target1,-1,tip_ori)
    for i=1,#joints do
        sim.setJointPosition(jh[i], sc[i])
    end
end

----------------------------------------------------------------------------------
------------------------Functions for Visualization-------------------------------
----------------------------------------------------------------------------------

--[[
Gets the Jointvalues headed over from C++-Environment.
Input:
    CCnt = Number of Configurations
    config = the values of the configuration
    inString = empty
    inBuff = empty
--]]
runConfig=function(CCnt, config, inString, InBuff)
    for i=1,#joints do
        local result = sim.setJointTargetPosition(jh[i],config[i])
    end
    setConfigEndPoint(config)
    publishValues()
    return {},{},{},''
end

--Function to round Values
function math.round(number, decimals, method)
    decimals = decimals or 0
    local factor = 10 ^ decimals
    if (method == "ceil" or method == "floor") then return math[method](number * factor) / factor
    else return tonumber(("%."..decimals.."f"):format(number)) end
end

function sysCall_threadmain()
    -- Initialization phase:
    joints = {  'KR120_2700_2_joint1','KR120_2700_2_joint2',
                'KR120_2700_2_joint3','KR120_2700_2_joint4',
                'KR120_2700_2_joint5','KR120_2700_2_joint6'}
    jh={-1,-1,-1,-1,-1,-1}
    jp={-1,-1,-1,-1,-1,-1}
    for i=1,#joints do
        jh[i]=sim.getObjectHandle("KR120_2700_2_joint" .. i)
        jp[i]=sim.getJointPosition(jh[i])
    end
    tip=sim.getObjectHandle("KR120_2700_2_tip")
    ikTarget=sim.getObjectHandle('ik_target')
    target1=sim.getObjectHandle('testTarget1')
    --target2=sim.getObjectHandle('testTarget2')

    ui = sim.getStringSignal("uisignal")
    if ui then
        ui = sim.unpackTable(ui)
    end

    while true do

    end
end