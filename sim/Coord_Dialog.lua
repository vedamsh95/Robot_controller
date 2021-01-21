--------------------Content of the UI--------------------
--[[
Save the values which are written in the left handside Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Editfield
    newValue = Content of the Editfield
--]]
function editcart(ui, id, newValue)
    edit_ids[id-999] = id
    editvalues[id] = newValue
end

--[[
Save the values which are written in the right handside Editfields.
Input:
    ui = Handler Value
    id = as Number of the Editfield
    newValue = Content of the Editfield
--]]
function editjp(ui, id, newValue)
    jpedit_ids[id-1999] = id
    jpeditvalues[id] = newValue
end

--[[
Setting up the Targetcoordinatesystem.
Building a JSON-String from the Input.
Sending the JSON-String to the C++-Environment via String Signal.
Receiving a String-Signal from the C++-Einvironment.
Decoding of that JSON-String and Ouput to the Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function applyDummy(ui,id)
    local comcnt = simUI.getComboboxItemCount(ui, 1013)
    for i=1,comcnt do
        simUI.removeComboboxItem(ui,1013,1)
    end
    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)
    local js
    local str
    if (simUI.getRadiobuttonValue(ui,1015)==1) then
        for key, value in pairs(editvalues) do
            editvalues[key] = simUI.getEditValue(ui, key)
        end
        local new_pos = {}
        for i= 1,3 do
            if (not edit_ids[i]) then
                new_pos[i] = 0
            else
                new_pos[i] = editvalues[edit_ids[i]]
            end
        end
        local new_ori = {}
        for i= 1,3 do
            if (not edit_ids[i+3]) then
                new_ori[i] = 0
            else
                new_ori[i] = math.rad(editvalues[edit_ids[i+3]])
            end
        end
        sim.setObjectPosition(ik_target, robot, new_pos)
        sim.setObjectOrientation(ik_target, robot, new_ori)
        js = {
            op = 1,
            data = {{
                m_a = tonumber(math.rad(new_ori[1])),
                m_b = tonumber(math.rad(new_ori[2])),
                m_c = tonumber(math.rad(new_ori[3])),
                m_x = tonumber(new_pos[1]),
                m_y = tonumber(new_pos[2]),
                m_z = tonumber(new_pos[3])
            }}
        }
        str = json.encode (js, { indent = true })
        sim.setStringSignal("callsignal",str)
    elseif (simUI.getRadiobuttonValue(ui,1016)==1) then
        for key, value in pairs(jpeditvalues) do
            jpeditvalues[key] = simUI.getEditValue(ui, key)
        end
        local new_jp = {}
        for i= 1,6 do
            if (not jpedit_ids[i]) then
                new_jp[i] = 0
            else
                new_jp[i] = jpeditvalues[jpedit_ids[i]]
            end
        end
        setConfigEndPoint(ui,new_jp)
        -- We want to only send data in the radiant format!
        for i = 1,6 do
            if simUI.getRadiobuttonValue(ui, 2007)==1 then
                new_jp[i] = math.rad(new_jp[i])
            end
        end
        js = {
            op = 0,
            data = {{
                j0 = tonumber(new_jp[1]),
                j1 = tonumber(new_jp[2]),
                j2 = tonumber(new_jp[3]),
                j3 = tonumber(new_jp[4]),
                j4 = tonumber(new_jp[5]),
                j5 = tonumber(new_jp[6])
            }}
        }
        str = json.encode (js, { indent = true })
        sim.setStringSignal("callsignal",str)
    end
end

function returnSignal()
    local ret = sim.getStringSignal("returnsignal")
    local ui = ui_1
    if ret then
        local obj, pos, err = json.decode (ret, 1, nil)
        --print(#obj.data)
        if err then
            sim.addStatusbarMessage("Error:", err)
        else
            if (obj.op==0) then
                if (simUI.getRadiobuttonValue(ui,2007)==1) then
                    simUI.setEditValue(ui,2000,tostring(math.deg(obj.data[1].j0)))
                    simUI.setEditValue(ui,2001,tostring(math.deg(obj.data[1].j1)))
                    simUI.setEditValue(ui,2002,tostring(math.deg(obj.data[1].j2)))
                    simUI.setEditValue(ui,2003,tostring(math.deg(obj.data[1].j3)))
                    simUI.setEditValue(ui,2004,tostring(math.deg(obj.data[1].j4)))
                    simUI.setEditValue(ui,2005,tostring(math.deg(obj.data[1].j5)))
                else
                    simUI.setEditValue(ui,2000,tostring(obj.data[1].j0))
                    simUI.setEditValue(ui,2001,tostring(obj.data[1].j1))
                    simUI.setEditValue(ui,2002,tostring(obj.data[1].j2))
                    simUI.setEditValue(ui,2003,tostring(obj.data[1].j3))
                    simUI.setEditValue(ui,2004,tostring(obj.data[1].j4))
                    simUI.setEditValue(ui,2005,tostring(obj.data[1].j5))
                end
                for i=1,#obj.data do
                    cconf[i] = {obj.data[i].j0,obj.data[i].j1,obj.data[i].j2,obj.data[i].j3,obj.data[i].j4,obj.data[i].j5}
                    simUI.insertComboboxItem(ui,1013,i,i..". Configuration")
                end
            elseif (obj.op==1) then
                simUI.setEditValue(ui,1000,tostring(obj.data[1].m_x))
                simUI.setEditValue(ui,1001,tostring(obj.data[1].m_y))
                simUI.setEditValue(ui,1002,tostring(obj.data[1].m_z))
                simUI.setEditValue(ui,1003,tostring(math.deg(obj.data[1].m_a)))
                simUI.setEditValue(ui,1004,tostring(math.deg(obj.data[1].m_b)))
                simUI.setEditValue(ui,1005,tostring(math.deg(obj.data[1].m_c)))
            end
        end
    else
        print("No Return-Signal. Try again or Restart the Programm!")
    end
    changeEnabled(ui,true)
    return {},{},{},''
end

--[[
Assisting function to set some Parts of the GUI enabled.
Input:
    enabled = Boolean
--]]
function changeEnabled(ui,enabled)
    simUI.setEnabled(ui,1007,enabled)
    simUI.setEnabled(ui,1008,enabled)
    simUI.setEnabled(ui,1009,enabled)
    simUI.setEnabled(ui,1010,enabled)
    if (simUI.getRadiobuttonValue(ui,1021)==1) then
        simUI.setEnabled(ui,1019,enabled)
    end
    simUI.setEnabled(ui,1011,enabled)
    movement_allowed = true
end

--[[
Assisting function to set the Targetcoordinatesystem with Jointconfigurations as Input.
Input:
    c = configuration
--]]
function setConfigEndPoint(ui,c)
    local sc = {}
    for i=1,#joints do
        sc[i] = sim.getJointPosition(jh[i])
    end
    for i=1,#joints do
        if (simUI.getRadiobuttonValue(ui,2007)==1) then
            sim.setJointPosition(jh[i], math.rad(c[i]))
        else
            sim.setJointPosition(jh[i], c[i])
        end
    end
    local tip_pos = sim.getObjectPosition(tip,-1)
    local tip_ori = sim.getObjectOrientation(tip, -1)
    sim.setObjectPosition(ik_dummy,-1,tip_pos)
    sim.setObjectOrientation(ik_dummy,-1,tip_ori)
    sim.setObjectPosition(ik_target,-1,tip_pos)
    sim.setObjectOrientation(ik_target,-1,tip_ori)
    local new_pos = sim.getObjectPosition(tip,robot)
    local new_ori = sim.getObjectOrientation(tip,robot)
    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui,999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui,999+i,tostring(math.round(math.deg(new_ori[i-3]),3)))
        end
    end
    for i=1,#joints do
        sim.setJointPosition(jh[i], sc[i])
    end
end

--[[
Button to send the Start- and the Targetconfiguration of the Manipulator to
the C++-Environment, where the Calculations should be done.
Setting up a JSON-String with the typ of Movement, the Start- and the Endconfiguration.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function CalculateIK(ui,id)

    if (simUI.getRadiobuttonValue(ui,1022)==1) then
        sendSplineData()
        return
    end

    local new_pos = sim.getObjectPosition(ik_target,robot)
    local new_ori = sim.getObjectOrientation(ik_target, robot)

    sim.setObjectPosition(ik_target, robot, new_pos)
    sim.setObjectOrientation(ik_target, robot, new_ori)

    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui,999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui,999+i,tostring(math.round(math.deg(new_ori[i-3]),3)))
        end
    end
    local curr_c = {}
    local tar_c = {}
    local js
    for i=1,#joints do
        curr_c[i] = sim.getJointPosition(jh[i])
    end
    for i=1,#joints do
        -- We want to only send data in the radiant format!
        if simUI.getRadiobuttonValue(ui, 2007)==1 then
            tar_c[i] = math.rad(simUI.getEditValue(ui,i+1999))
        else
            tar_c[i] = simUI.getEditValue(ui,i+1999)
        end
    end
    if (simUI.getRadiobuttonValue(ui,1007) == 1) then
        if (simUI.getRadiobuttonValue(ui,1010) == 1) then
            js = {
                op = 2,
                data = {{
                    j0 = tonumber(curr_c[1]),
                    j1 = tonumber(curr_c[2]),
                    j2 = tonumber(curr_c[3]),
                    j3 = tonumber(curr_c[4]),
                    j4 = tonumber(curr_c[5]),
                    j5 = tonumber(curr_c[6])
                },
                    {
                        j0 = tonumber(tar_c[1]),
                        j1 = tonumber(tar_c[2]),
                        j2 = tonumber(tar_c[3]),
                        j3 = tonumber(tar_c[4]),
                        j4 = tonumber(tar_c[5]),
                        j5 = tonumber(tar_c[6])
                    }}
            }
        elseif (simUI.getRadiobuttonValue(ui,1009) == 1) then
            js = {
                op = 3,
                data = {{
                    j0 = tonumber(curr_c[1]),
                    j1 = tonumber(curr_c[2]),
                    j2 = tonumber(curr_c[3]),
                    j3 = tonumber(curr_c[4]),
                    j4 = tonumber(curr_c[5]),
                    j5 = tonumber(curr_c[6])
                },
                    {
                        j0 = tonumber(tar_c[1]),
                        j1 = tonumber(tar_c[2]),
                        j2 = tonumber(tar_c[3]),
                        j3 = tonumber(tar_c[4]),
                        j4 = tonumber(tar_c[5]),
                        j5 = tonumber(tar_c[6])
                    }}
            }
        end
    elseif (simUI.getRadiobuttonValue(ui,1008) == 1) then
        js = {
            op = 4,
            data = {{
                j0 = tonumber(curr_c[1]),
                j1 = tonumber(curr_c[2]),
                j2 = tonumber(curr_c[3]),
                j3 = tonumber(curr_c[4]),
                j4 = tonumber(curr_c[5]),
                j5 = tonumber(curr_c[6])
            },
                {
                    j0 = tonumber(tar_c[1]),
                    j1 = tonumber(tar_c[2]),
                    j2 = tonumber(tar_c[3]),
                    j3 = tonumber(tar_c[4]),
                    j4 = tonumber(tar_c[5]),
                    j5 = tonumber(tar_c[6])
                }},
            vel = tonumber(simUI.getEditValue(ui_1, 3020)),   -- The velocity for the lin movement
            acc = tonumber(simUI.getEditValue(ui_1, 3021)),   -- The acceleration for the lin movement
        }
    end
    local str = json.encode (js, { indent = true })
    sim.setStringSignal("callsignal",str)
end

--[[
This function sends the points for the spline movement to the controller.
The first point is the current position of the robot. The follwoing points
are from the combobox. The orientation is constant and stays the same during
the movement.
--]]
function sendSplineData()

    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)

    local curr_c = {}
    for i=1,#joints do
        curr_c[i] = sim.getJointPosition(jh[i])
    end

    start_config = {
        j0 = tonumber(curr_c[1]),
        j1 = tonumber(curr_c[2]),
        j2 = tonumber(curr_c[3]),
        j3 = tonumber(curr_c[4]),
        j4 = tonumber(curr_c[5]),
        j5 = tonumber(curr_c[6])
    }

    data_arr = {{
        m_a = tip_ori[1],
        m_b = tip_ori[2],
        m_c = tip_ori[3],
        m_x = tip_pos[1],
        m_y = tip_pos[2],
        m_z = tip_pos[3]
    } }

    local points = #spline_points_raw
    for i=1,points do
        data_arr[i+1] = {
            m_a = tip_ori[1],
            m_b = tip_ori[2],
            m_c = tip_ori[3],
            m_x = spline_points_raw[i][1],
            m_y = spline_points_raw[i][2],
            m_z = spline_points_raw[i][3]
        }
    end

    js = {
        op = 5,
        data = data_arr,
        vel = tonumber(simUI.getEditValue(ui_1, 3020)),   -- The velocity for the spline movement
        acc = tonumber(simUI.getEditValue(ui_1, 3021)),   -- The acceleration for the spline movement
        start_config = start_config
    }

    local str = json.encode (js, { indent = true })
    sim.setStringSignal("callsignal",str)

end

--[[
Change the Movement Mode and allow to use sync or async Movement.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function switchMVMode(ui,id)
    if (id==1007) then
        simUI.setEnabled(ui,1009,true)
        simUI.setEnabled(ui,1010,true)
        simUI.setRadiobuttonValue(ui,1009,1)
    else
        simUI.setEnabled(ui,1009,false)
        simUI.setEnabled(ui,1010,false)
    end
end

--[[
Use that funtion to switch between the different Configurations that where calcutated for
a target point in the C++-Environment.
Input:
    ui = UI Handler Value
    id = as Number of the Combobox
    newValue = Content of the Combobox
--]]
function switchConfig(ui,id,newValue)
    if (newValue ~= 0) then
        for i=1,6 do
            if (simUI.getRadiobuttonValue(ui,2007)==1) then
                simUI.setEditValue(ui,1999+i,tostring(math.deg(cconf[newValue][i])))
                editjp(ui,1999+i,tostring(math.deg(cconf[newValue][i])))
            else
                simUI.setEditValue(ui,1999+i,tostring(cconf[newValue][i]))
                editjp(ui,1999+i,tostring(cconf[newValue][i]))
            end
        end
        setConfigEndPoint(ui,cconf[newValue])
    end
end

--[[
Change between Cartesian Mode or Joint Configuration Mode.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function switchOPMode(ui,id)
    if (id==1015) then
        simUI.setEnabled(ui,1017,true)
        simUI.setEnabled(ui,1018,false)
    else
        simUI.setEnabled(ui,1017,false)
        simUI.setEnabled(ui,1018,true)
    end
end

--[[
Change between Normal movements or Spline movement.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function switchSplineMode(ui,id)
    if (id==1021) then
        if (movement_allowed) then
            simUI.setEnabled(ui,1019,true)
        else
            simUI.setEnabled(ui,1011,false)
        end
        simUI.setEnabled(ui,1020,false)
    else
        simUI.setEnabled(ui,1019,false)
        simUI.setEnabled(ui,1020,true)
        simUI.setEnabled(ui,1011,true)
    end
end

--[[
Change between Radian and Degrees in the right handside Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function radiobuttonClick(ui,id)
    if (id==2007) then
        for i=1,6 do
            local value = simUI.getEditValue(ui,i+1999)
            simUI.setEditValue(ui,i+1999,tostring(math.deg(value)))
        end
    else
        for i=1,6 do
            local value = simUI.getEditValue(ui,i+1999)
            simUI.setEditValue(ui,i+1999,tostring(math.rad(value)))
        end
    end
end

--[[
Handle the combobox for the spline functionality
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
    value   = the index of the selected value starting at 0!
--]]
function splineSwitchPoint(ui,id,value)
    splineCancel(ui,3004)
end

--[[
Converts the stored points to texts and shows them in the combobox
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
    selected_index = The index that should be selected afterwards in range [1..N]
--]]
function createSplinePointsText(ui,id,selected_index)
    local size = #spline_points_raw
    local texts = {}

    for i=1,size do
        local coords = spline_points_raw[i]
        local text = string.format("%s: %f, %f, %f", string.char(64+i), coords[1], coords[2], coords[3])
        texts[i] = text
    end

    simUI.setComboboxItems(ui,id,texts,selected_index-1)
    splineCancel(ui,3004)
    updatePath(pathHandle, spline_points_raw)
end

--[[
Shows the current spline point in the edit fields
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function splineCancel(ui,id)
    local index = simUI.getComboboxSelectedIndex(ui,3000) + 1
    local coords = spline_points_raw[index]
    if (#spline_points_raw == 0) then
        coords = {0, 0, 0}
    end
    for i=1,3 do
        simUI.setEditValue(ui,3000+i, tostring(coords[i]))
    end
end

--[[
Changes the currently selected spline point to the values in the edit fields
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function splineApply(ui,id)
    local index = simUI.getComboboxSelectedIndex(ui,3000) + 1
    local coords = {}
    for i=1,3 do
       coords[i] = math.round(simUI.getEditValue(ui,3000+i), 3)
    end
    spline_points_raw[index] = coords
    createSplinePointsText(ui,3000,index)
end

--[[
Inserts a new point after the currently selected one with the values in the
edit fields.
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function splineInsert(ui,id)
    local size = #spline_points_raw
    local index = simUI.getComboboxSelectedIndex(ui,3000) + 1
    local coords = {}
    for i=1,3 do
        coords[i] = math.round(simUI.getEditValue(ui,3000+i), 3)
    end
    -- Move all further entries by one and insert the new point
    if (index<size) then
        for i=size,index+1,-1 do
            spline_points_raw[i+1] = spline_points_raw[i]
        end
        spline_points_raw[index+1] = coords
    else
        spline_points_raw[size+1] = coords
    end
    createSplinePointsText(ui,3000,index+1)
end

--[[
Deletes the currently selected spline point
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function splineDelete(ui,id)
    local size = #spline_points_raw
    local index = simUI.getComboboxSelectedIndex(ui,3000) + 1
    local selected = index
    if (index==size) then
        selected = index-1
    else
        for i=index,size-1 do
            spline_points_raw[i] = spline_points_raw[i+1]
        end
    end
    spline_points_raw[size] = nil
    createSplinePointsText(ui,3000,selected)
end

--[[
Exorts the current spline points to a csv file with the given name. If the file
already exists, it will be overriden.
Input:
    filename    = Name of the file without the ending!
--]]
function exportSplineCSV(filename)
    local text = "x,y,z\n"
    local rows = {}
    for i=1,#spline_points_raw do
        rows[i] = string.format("%f,%f,%f", spline_points_raw[i][1], spline_points_raw[i][2], spline_points_raw[i][3])
    end
    text = text .. table.concat(rows, "\n")

    local scenePath = sim.getStringParameter(sim.stringparam_scene_path)
    local file = io.open(scenePath .. "/" .. filename .. ".csv", "w")
    file:write(text)
    file:close()
end

--[[
Splits the given string at the given seperator and returns the substring
as a table.
Input:
    inputStr    = The string to split
    sep         = The separator string
--]]
function splitString(inputstr, sep)
    local parts = {}
    for str in string.gmatch(inputstr, "([^"..sep.."]+)") do
        table.insert(parts, str)
    end
    return parts
end

--[[
Imports a csv file and stores the result in the spline_points_raw table.
Input:
    filename    = The name of the file to import (Without ending!)
--]]
function importSplineCSV(filename)
    local scenePath = sim.getStringParameter(sim.stringparam_scene_path)
    local file = io.open(scenePath .. "/" .. filename .. ".csv", "r")
    local raw_points = {}
    for line in file:lines() do
        local slices = splitString(line, ",")
        table.insert(raw_points, slices)
    end

    for k in pairs(spline_points_raw) do
        spline_points_raw[k] = nil
    end

    for i=2,#raw_points do
        table.insert(spline_points_raw, {
            tonumber(raw_points[i][1]),
            tonumber(raw_points[i][2]),
            tonumber(raw_points[i][3])
        })
    end

    file:close()

    createSplinePointsText(ui_1, 3000, #spline_points_raw)
end

--[[
This functions handles the ui events of the csv functionality window.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function splineIO(ui, id)

    -- CSV functionality button
    if (id == 3008) then
        local directory = sim.getStringParameter(sim.stringparam_scene_path) .."/"
        local i, names, popen = 0, {}, io.popen
        command = ""
        if (package.config:sub(1,1) == '/') then
            command = 'ls -a "'..directory..'"'
        else
            command = 'dir "'..directory..'" /b'
        end
        local pfile = popen(command)
        for filename in pfile:lines() do
            local parts = splitString(filename, ".")
            if (parts[2] == "csv") then
                i = i + 1
                names[i] = parts[1]
            end
        end
        if (#names > 0) then
            simUI.setComboboxItems(ui_3,5000,names,0)
        end
        pfile:close()
        simUI.show(ui_3)
        return
    end

    -- Import button
    if (id == 5001) then
        local selected = simUI.getComboboxSelectedIndex(ui,5000)
        if (selected >= 0) then -- ui indices start at 0
           local name = simUI.getComboboxItemText(ui,5000,selected)
            importSplineCSV(name)
        end
        --local name = simUI.get
        simUI.hide(ui_3)
        return
    end

    -- Export button
    if (id == 5003) then
        local filename = simUI.getEditValue(ui, 5002)
        filename = filename:match'^%s*(.*%S)' or '' -- trim
        if (#filename > 0) then
            filename = splitString(filename, ".")[1]
            exportSplineCSV(filename)
            simUI.hide(ui_3)
        end
        return
    end

    -- Cancel button
    if (id == 5004) then
        simUI.hide(ui_3)
    end
end

--[[
This functions updates the path for the spline functionality.
Input:
    handle      = Handle of the path that should be updated
    id          = Points for the path, the first point however will be the current position
--]]
function updatePath(handle, values)

    if (#values == 0) then
        sim.cutPathCtrlPoints(handle, -1, 0)
        return
    end

    local data = {}

    -- Insert the current position as the first point
    tip_pos = sim.getObjectPosition(tip,robot)
    table.insert(data, tip_pos[1])
    table.insert(data, tip_pos[2])
    table.insert(data, tip_pos[3])
    for j=1,8 do
        table.insert(data, 0)
    end

    -- Fill the table with the missing values
    for i=1,#values do
        table.insert(data, values[i][1])
        table.insert(data, values[i][2])
        table.insert(data, values[i][3])
        for j=1,8 do
            table.insert(data, 0)
        end
    end

    sim.cutPathCtrlPoints(handle, -1, 0)
    sim.insertPathCtrlPoints(handle, 0, 0, #values+1, data)
end

---------------------------------------------

--Close the UI
function closeEventHandler(h)
    simUI.hide(h)
end

--Close the second UI by hitting the Button
function buttonok(ui,id)
    simUI.hide(ui_2)
    simUI.show(ui_1)
end

--Function to round Values
function math.round(number, decimals, method)
    decimals = decimals or 0
    local factor = 10 ^ decimals
    if (method == "ceil" or method == "floor") then return math[method](number * factor) / factor
    else return tonumber(("%."..decimals.."f"):format(number)) end
end

--Initialize the UI
if (sim_call_type==sim.syscb_init) then

    coord_dialog = [[<ui closeable="true" on-close="closeEventHandler" layout="hbox" title="Coordinates" resizable="true">
    <group layout="vbox">
        <label text="Choose Operation Mode for the Robot Manipulator."></label>
        <group layout="hbox">
            <radiobutton text="By Cartisian Coordinates" checked="true" on-click="switchOPMode" id="1015" />
            <radiobutton text="By Joint Angles" on-click="switchOPMode" id="1016" />
        </group>
        <group layout="hbox">
            <group layout="vbox" id="1017">
                <label text="Please enter coordinates(in Meter) and orientation(in Degree)."></label>
                <group layout="hbox">
                    <group>
                        <label text="X:"></label>
                        <edit id="1000" value="0" on-editing-finished="editcart"></edit>
                    </group>
                    <group>
                        <label text="Y:"></label>
                        <edit id="1001" value="0" on-editing-finished="editcart"></edit>
                    </group>
                    <group>
                        <label text="Z:"></label>
                        <edit id="1002" value="0" on-editing-finished="editcart"></edit>
                    </group>
                </group>
                <group layout="hbox">
                    <group>
                        <label text="A:"></label>
                        <edit id="1003" value="0" on-editing-finished="editcart"></edit>
                    </group>
                    <group>
                        <label text="B:"></label>
                        <edit id="1004" value="0" on-editing-finished="editcart"></edit>
                    </group>
                    <group>
                        <label text="C:"></label>
                        <edit id="1005" value="0" on-editing-finished="editcart"></edit>
                    </group>
                </group>
            </group>
            <group layout="vbox">
                <label text="Please enter the Joint Angles you want to change.
You could either use Radian or Degree as Input."></label>
                <group layout="hbox">
                    <radiobutton text="Degree" on-click="radiobuttonClick" id="2007" />
                    <radiobutton text="Radian" on-click="radiobuttonClick" id="2008" />
                </group>
                <group layout="vbox" enabled="false" id="1018">
                    <group layout="hbox">
                        <group layout="vbox">
                            <label text="Theta 1:"></label>
                            <edit id="2000" value="0" on-editing-finished="editjp"></edit>
                        </group>
                        <group layout="vbox">
                            <label text="Theta 2:"></label>
                            <edit id="2001" value="0" on-editing-finished="editjp"></edit>
                        </group>
                        <group layout="vbox">
                            <label text="Theta 3:"></label>
                            <edit id="2002" value="0" on-editing-finished="editjp"></edit>
                        </group>
                    </group>
                    <group layout="hbox">
                        <group layout="vbox">
                            <label text="Theta 4:"></label>
                            <edit id="2003" value="0" on-editing-finished="editjp"></edit>
                        </group>
                        <group layout="vbox">
                            <label text="Theta 5:"></label>
                            <edit id="2004" value="0" on-editing-finished="editjp"></edit>
                        </group>
                        <group layout="vbox">
                            <label text="Theta 6:"></label>
                            <edit id="2005" value="0" on-editing-finished="editjp"></edit>
                        </group>
                    </group>
                </group>
                <label text="Please select the Configuration of the Endeffector."></label>
                <combobox id="1013" on-change="switchConfig"></combobox>
            </group>
        </group>
        <group layout = "vbox">
            <button text="Apply" id="1006" onclick="applyDummy"></button>
        </group>
    </group>
    <group layout="vbox">
        <label text="Please select an operation mode."></label>
        <group layout="hbox">
            <radiobutton text="Normal" checked="true"  on-click="switchSplineMode" id="1021" />
            <radiobutton text="Spline" checked="false" on-click="switchSplineMode" id="1022" />
        </group>
        <group layout="vbox" enabled="true" id="1019">
            <label text="Normal movements:"></label>
            <group layout="hbox">
                <radiobutton text="PTP" enabled="false" checked="true" on-click="switchMVMode" id="1007" />
                <radiobutton text="LIN" enabled="false" on-click="switchMVMode" id="1008" />
            </group>
            <group layout="hbox">
                <radiobutton text="sync" checked="true" enabled="false" on-click="" id="1009" />
                <radiobutton text="async" enabled="false" on-click="" id="1010" />
            </group>
        </group>
        <group layout="vbox">
            <label text="Please enter a velocity [m/s] and an acceleration [m/s^2]
for the lin and spline movement:"></label>
            <group layout="hbox">
                <group>
                    <label text="Velocity:"></label>
                    <edit id="3020" value="0"></edit>
                </group>
                <group>
                    <label text="Acceleration:"></label>
                    <edit id="3021" value="0"></edit>
                </group>
            </group>
        </group>
        <group layout="vbox" id="1020" enabled="false">
            <label text="Spline functionality:"></label>
            <combobox id="3000" on-change="splineSwitchPoint"></combobox>
            <group layout="hbox">
                <group>
                    <label text="X:"></label>
                    <edit id="3001" value="0"></edit>
                </group>
                <group>
                    <label text="Y:"></label>
                    <edit id="3002" value="0"></edit>
                </group>
                <group>
                    <label text="Z:"></label>
                    <edit id="3003" value="0"></edit>
                </group>
            </group>
            <group layout="grid">
                <button text="Cancel" id="3004" onclick="splineCancel"></button>
                <button text="Delete" id="3005" onclick="splineDelete"></button>
                <br />
                <button text="Apply" id="3006" onclick="splineApply"></button>
                <button text="Insert" id="3007" onclick="splineInsert"></button>
            </group>
            <group>
                <button text="CSV functionality" id="3008" onclick="splineIO"></button>
            </group>

        </group>
        <button text="Calculate and Move" id="1011" enabled="false" onclick="CalculateIK"></button>
    </group>
</ui>]]

    error = [[<ui closeable="false" on-close="buttonok" layout="vbox" title="Error">
	<label text="For the Values entered, is no path available."></label>
	<button text="OK" onclick="buttonok"></button>
</ui>]]

    splineIO = [[<ui closeable="false" on-close="splineIO" layout="vbox" title="CSV functionality">
     <label text="You can either import spline points from a vsc file
or export the current points to a file."></label>
    <group layout="hbox">
        <group layout="vbox">
            <label text="Import"></label>
            <combobox id="5000" on-change="splineIO"></combobox>
            <button text="Import" onclick="splineIO" id="5001"></button>
        </group>
        <group layout="vbox">
            <label text="Export"></label>
            <edit id="5002" value=""></edit>
            <button text="Export" onclick="splineIO" id="5003"></button>
        </group>
    </group>
    <button text="Cancel" onclick="splineIO" id="5004"></button>
</ui>]]

    movement_allowed = false    -- Whether apply has been pressed once
                                -- and the normal movement is allowed

    --spline_points_raw = {{0.1, 0.3, 0.4}, {0.5,0.6,0.0}, {0.6, 0.9, 1.0}, {0.33, 0.33, 0.33}}
    spline_points_raw = {}

    edit_ids = {}
    editvalues = {}
    jpedit_ids = {}
    jpeditvalues = {}
    c = {}
    cconf={}

    json=require("dkjson")

    ui_1=simUI.create(coord_dialog)
    ui_2=simUI.create(error)
    ui_3=simUI.create(splineIO)

    local myuis = {ui_1,ui_2,ui_3}
    sim.setStringSignal("uisignal",sim.packTable(myuis))

   -- Spline
    pathIntParams = { 7, 0, 0 }                                         -- First one is the size of the path
    pathColor = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }                  -- Color of the path
    pathHandle = sim.createPath(-1, pathIntParams, nullptr, pathColor)
    simUI.setEditValue(ui_1, 3020, tostring(1.0))
    simUI.setEditValue(ui_1, 3021, tostring(5.0))

    ik_dummy = sim.getObjectHandle('ik_target')
    ik_target = sim.getObjectHandle('testTarget1')
    ik_test = sim.getObjectHandle('testTarget2')
    tip = sim.getObjectHandle('KR120_2700_2_tip')
    robot = sim.getObjectHandle('Graph')
    joints = {  'KR120_2700_2_joint1','KR120_2700_2_joint2',
        'KR120_2700_2_joint3','KR120_2700_2_joint4',
        'KR120_2700_2_joint5','KR120_2700_2_joint6'}
    jh={-1,-1,-1,-1,-1,-1}
    jp={-1,-1,-1,-1,-1,-1}
    for i=1,#joints do
        jh[i]=sim.getObjectHandle("KR120_2700_2_joint" .. i)
        jp[i]=sim.getJointPosition(jh[i])
    end
    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)
    sim.setObjectPosition(ik_dummy, robot, tip_pos)
    sim.setObjectOrientation(ik_dummy, robot, tip_ori)
    sim.setObjectPosition(ik_target, robot, tip_pos)
    sim.setObjectOrientation(ik_target, robot, tip_ori)
    simUI.hide(ui_2)
    simUI.hide(ui_3)
    simUI.setRadiobuttonValue(ui_1,2008,1)

    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui_1,999+i,tostring(math.round(tip_pos[i],3)))
            editcart(ui_1,999+i,tostring(math.round(tip_pos[i],3)))
        else
            simUI.setEditValue(ui_1,999+i,tostring(math.round(math.deg(tip_ori[i-3]),3)))
            editcart(ui_1,999+i,tostring(math.round(math.deg(tip_ori[i-3]),3)))
        end
    end
    for i=1,6 do
        simUI.setEditValue(ui_1,1999+i,tostring(math.round(jp[i],3)))
        editjp(ui_1,1999+i,tostring(math.round(jp[i],3)))
    end
    simUI.insertComboboxItem(ui_1,1013,0,"Select a config:")
end

--Close all UI's at the end of the simulation
if (sim_call_type==sim.syscb_cleanup) then
    simUI.destroy(ui_1)
    simUI.destroy(ui_2)
    simUI.destroy(ui_3)
    sim.removeObjectFromSelection(sim.handle_single, ik_test)
end