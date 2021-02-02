--------------------Content of the UI--------------------
--[[
Save the values which are written in the left handside Editfields.
Input:
    ui = UI Handler Value
    id = as Number of the Editfield
    newValue = Content of the Editfield
--]]
function editcart(_, id, newValue)
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
function editjp(_, id, newValue)
    jpedit_ids[id-1999] = id
    jpeditvalues[id] = newValue
end

--[[
Setting up the Targetcoordinatesystem.
Building a JSON-String from the Input.
Sending the JSON-String to the C++-Environment via String Signal.
Input:
    ui = UI Handler Value
    id = as Number of the Button
--]]
function applyDummy(ui,_)
    local comcnt = simUI.getComboboxItemCount(ui, 1013)
    for _=1,comcnt do
        simUI.removeComboboxItem(ui,1013,1)
    end
    tip_pos = sim.getObjectPosition(tip,robot)
    tip_ori = sim.getObjectOrientation(tip,robot)
    local js
    local str
    if (simUI.getRadiobuttonValue(ui,1015)==1) then
        for key, _ in pairs(editvalues) do
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
        set_orientation(ik_target, robot, new_ori)
        js = {
            op = 1,
            data = {{
                m_a = tonumber(new_ori[1]),
                m_b = tonumber(new_ori[2]),
                m_c = tonumber(new_ori[3]),
                m_x = tonumber(new_pos[1]),
                m_y = tonumber(new_pos[2]),
                m_z = tonumber(new_pos[3])
            }}
        }
        str = json.encode (js, { indent = true })
        sim.setStringSignal("callsignal",str)
    elseif (simUI.getRadiobuttonValue(ui,1016)==1) then
        for key, _ in pairs(jpeditvalues) do
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
        -- We want to only send data in the radiant format!
        for i = 1,6 do
            if simUI.getRadiobuttonValue(ui, 2007)==1 then
                new_jp[i] = math.rad(new_jp[i])
            end
        end
        setConfigEndPoint(ui,new_jp)
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

--[[
Receiving a String-Signal from the C++-Environment.
Decoding of that JSON-String and Output to the Editfields.
This function gets be called by the controller.
--]]
function returnSignal()
    local ret = sim.getStringSignal("returnsignal")
    local ui = ui_1
    if ret then
        local obj, _, err = json.decode (ret, 1, nil)
        --print(#obj.data)
        if err then
            sim.addStatusbarMessage("Error:", err)
        elseif obj.data == nil then
            error_set("No result possible for the entered values!")
            error_show()
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
                switchConfig(ui_1, 1013, 1)
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
        sim.setJointPosition(jh[i], c[i])
    end
    local tip_pos = sim.getObjectPosition(tip,-1)
    local tip_ori = sim.getObjectOrientation(tip, -1)
    sim.setObjectPosition(ik_dummy,-1,tip_pos)
    sim.setObjectOrientation(ik_dummy,-1,tip_ori)
    sim.setObjectPosition(ik_target,-1,tip_pos)
    sim.setObjectOrientation(ik_target,-1,tip_ori)
    local new_pos = sim.getObjectPosition(tip,robot)
    --local new_ori = sim.getObjectOrientation(tip,robot)
    local new_ori = get_orientation(tip, robot)
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
function CalculateIK(ui,_)

    if (simUI.getRadiobuttonValue(ui,1022)==1) then
        sendSplineData()
        return
    end

    local new_pos = sim.getObjectPosition(ik_target,robot)
    local new_ori = sim.getObjectOrientation(ik_target, robot)
    local our_ori = get_orientation(ik_target, robot)

    sim.setObjectPosition(ik_target, robot, new_pos)
    sim.setObjectOrientation(ik_target, robot, new_ori)

    for i=1,6 do
        if i <=3 then
            simUI.setEditValue(ui,999+i,tostring(math.round(new_pos[i],3)))
        else
            simUI.setEditValue(ui,999+i,tostring(math.round(math.deg(our_ori[i-3]),3)))
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
    tip_ori = get_orientation(tip, robot)

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
        type = simUI.getComboboxSelectedIndex(ui_3, 5005),
        elong = tonumber(simUI.getEditValue(ui_3, 5006)),
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
        -- PTP Mode
        simUI.setEnabled(ui,1009,true)
        simUI.setEnabled(ui,1010,true)
        simUI.setRadiobuttonValue(ui,1009,1)
        lin_path_hide(true)
    else
        -- LIN Mode
        simUI.setEnabled(ui,1009,false)
        simUI.setEnabled(ui,1010,false)
    end
end

--[[
Use that function to switch between the different Configurations that where calculated for
a target point in the C++-Environment.
Input:
    ui = UI Handler Value
    id = as Number of the Combobox
    newValue = Content of the Combobox
--]]
function switchConfig(ui,_,newValue)
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
        -- Normal mode
        if (movement_allowed) then
            simUI.setEnabled(ui,1019,true)
        else
            simUI.setEnabled(ui,1011,false)
        end
        simUI.setEnabled(ui,1020,false)
        spline_path_hide(true)
    else
        -- Spline Mode
        simUI.setEnabled(ui,1019,false)
        simUI.setEnabled(ui,1020,true)
        simUI.setEnabled(ui,1011,true)
        spline_path_hide(false)
        lin_path_hide(true)
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
        if not degree then
            for i=1,6 do
                local value = simUI.getEditValue(ui,i+1999)
                simUI.setEditValue(ui,i+1999,tostring(math.deg(value)))
            end
            degree = true
        end
    else
        if degree then
            for i=1,6 do
                local value = simUI.getEditValue(ui,i+1999)
                simUI.setEditValue(ui,i+1999,tostring(math.rad(value)))
            end
            degree = false
        end
    end
end

--------------------Spline UI functionality--------------------

----- IO Handler

--[[
If the user switches to another point in the spline combobox, the values in
the edit fields should be changed accordingly
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function ui_spline_combobox(ui, id)
    if id ~= UI_IDs.SPLINE.COMBOBOX then
        return
    end

    ui_spline_cancel(ui,UI_IDs.SPLINE.CANCEL)
end

--[[
Resets the edit field to the currently selected point.
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function ui_spline_cancel(ui, id)
    if id ~= UI_IDs.SPLINE.CANCEL then
        return
    end

    local index = simUI.getComboboxSelectedIndex(ui,UI_IDs.SPLINE.COMBOBOX) + 1
    local coords = spline_points_raw[index]
    if (#spline_points_raw == 0) then
        coords = {0, 0, 0}
    end
    for i=0,2 do
        simUI.setEditValue(ui,UI_IDs.SPLINE.X+i, tostring(coords[i+1]))
    end
end

--[[
Deletes the currently selected spline point. The next point in the list will be
active. If the last point got deleted, the previous point gets selected. The edit
fields will also bet set to the corresponding values.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function ui_spline_delete(ui, id)
    if id ~= UI_IDs.SPLINE.DELETE then
        return
    end

    local size = #spline_points_raw
    local index = simUI.getComboboxSelectedIndex(ui,UI_IDs.SPLINE.COMBOBOX) + 1
    local selected = index
    if (index==size) then
        -- The last point got deleted
        selected = index-1
    else
        -- Move the following points
        for i=index,size-1 do
            spline_points_raw[i] = spline_points_raw[i+1]
        end
    end
    spline_points_raw[size] = nil
    show_spline_points(ui,UI_IDs.SPLINE.COMBOBOX,selected)
end

--[[
Changes the currently selected spline point to the values in the edit fields.
If there is no point yet, the button will call the insert button.
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function ui_spline_apply(ui, id)
    if id ~= UI_IDs.SPLINE.APPLY then
        return
    end

    local index = simUI.getComboboxSelectedIndex(ui,UI_IDs.SPLINE.COMBOBOX) + 1    -- Start index conversion!
    if (index == 0) then
        -- In case there is no entry yet, the apply button acts as the insert button
        splineInsert(ui, UI_IDs.SPLINE.INSERT)
    else
        local coords = {}
        for i=0,2 do
            coords[i+1] = math.round(simUI.getEditValue(ui,UI_IDs.SPLINE.X+i), 3)
        end
        spline_points_raw[index] = coords
        show_spline_points(ui,UI_IDs.SPLINE.COMBOBOX,index)
    end
end

--[[
Inserts a new point after the currently selected one with the values in the
edit fields.
Input:
    ui      = UI Handler value
    id      = as Number of the combobox
--]]
function ui_spline_insert(ui, id)
    if id ~= UI_IDs.SPLINE.INSERT then
        return
    end

    local size = #spline_points_raw
    local index = simUI.getComboboxSelectedIndex(ui,UI_IDs.SPLINE.COMBOBOX) + 1
    local coords = {}
    for i=0,2 do
        coords[i+1] = math.round(simUI.getEditValue(ui,UI_IDs.SPLINE.X+i), 3)
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
    show_spline_points(ui,UI_IDs.SPLINE.COMBOBOX,index+1)
end

--[[
This functions opens the advanced settings for the spline movement and
fills the import file selection combobox.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function ui_spline_advanced(_, id)
    if id ~= UI_IDs.SPLINE.ADVANCED then
        return
    end

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
        simUI.setComboboxItems(ui_3,UI_IDs.ADVANCED.IMPORT_CB,names,0)
    end
    pfile:close()
    simUI.show(ui_3)
    return
end

--[[
This function imports the currently selected csv file and hides the
advanced spline settings ui afterwards.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function ui_advanced_import(ui, id)
    if id ~= UI_IDs.ADVANCED.IMPORT_BT then
        return
    end

    local selected = simUI.getComboboxSelectedIndex(ui,UI_IDs.ADVANCED.IMPORT_CB)
    if (selected >= 0) then -- ui indices start at 0, -1 means none is selected
        local name = simUI.getComboboxItemText(ui,UI_IDs.ADVANCED.IMPORT_CB,selected)
        raw_points = import_spline_CSV(name)
        -- Delete the current table and insert the new values
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
        show_spline_points(ui_1, UI_IDs.SPLINE.COMBOBOX, #spline_points_raw)
        -- Close the ui so that one does not have to press the ok button
        simUI.hide(ui)
    end
    return
end

--[[
This function exports the current spline points to a csv file with the
given name and hides the advanced spline settings ui afterwards.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function ui_advanced_export(ui, id)
    if id ~= UI_IDs.ADVANCED.EXPORT_BT then
        return
    end

    local filename = simUI.getEditValue(ui, UI_IDs.ADVANCED.EXPORT_ED)
    filename = filename:match'^%s*(.*%S)' or '' -- trim
    if (#filename > 0) then
    filename = splitString(filename, ".")[1]
    export_spline_CSV(filename, spline_points_raw)
    -- Close the ui so that one does not have to press the ok button
    simUI.hide(ui)
    end
    return
    end

--[[
This functions hides the advanced spline settings ui. It gets called by
the custom ui plugin when the corresponding button is pressed.
Input:
    ui      = UI Handler value
    id      = as Number of the Button
--]]
function ui_advanced_ok(ui, id)
    if id == UI_IDs.ADVANCED.OK then
        simUI.hide(ui)
    end
end

----- Helper functions

--[[
Imports a csv file that got exported using the function exportSplineCSV.
It will return the table with the read in points.
Input:
    filename    = The name of the file to import (Without ending or path!)
Output:
    table       = A table containing table3 objects denoting the points
--]]
function import_spline_CSV(filename)
    local scenePath = sim.getStringParameter(sim.stringparam_scene_path)
    local file = io.open(scenePath .. "/" .. filename .. ".csv", "r")
    local raw_points = {}
    for line in file:lines() do
        local slices = splitString(line, ";")
        table.insert(raw_points, slices)
    end
    file:close()
    return raw_points
end

--[[
Exports the given points to a csv file with the given name. If the file
already exists, it will be overridden.
Input:
    filename    = Name of the file without the ending!
    points      = A table containing of table3 with numbers
--]]
function export_spline_CSV(filename, points)
    local text = "x,y,z\n"
    local rows = {}
    for i=1,#points do
        rows[i] = string.format("%f;%f;%f", points[i][1], points[i][2], points[i][3])
    end
    text = text .. table.concat(rows, "\n")
    local scenePath = sim.getStringParameter(sim.stringparam_scene_path)
    local file = io.open(scenePath .. "/" .. filename .. ".csv", "w")
    file:write(text)
    file:close()
end

--[[
Converts the stored points to texts and shows them in the specified combobox.
Input:
    ui      = The ui handle the combobox is in
    id      = the id of the combobox
    selected_index = The index that should be selected afterwards in range [1..N]
--]]
function show_spline_points(ui, id, selected_index)
    local size = #spline_points_raw
    local texts = {}
    for i=1,size do
        local coords = spline_points_raw[i]
        local text = string.format("%s: %f, %f, %f", string.char(64+i), coords[1], coords[2], coords[3])
        texts[i] = text
    end
    simUI.setComboboxItems(ui,id,texts,selected_index-1)
    ui_spline_cancel(ui,UI_IDs.SPLINE.CANCEL)
    update_spline_path(spline_path_handle, spline_points_raw)
end

function spline_init()

    -- Default values
    local path_size = 5                 -- The line thickness of the path
    local path_color_r = 0              -- The red   component of the paths color
    local path_color_g = 1              -- The green component of the paths color
    local path_color_b = 0              -- The blue  component of the paths color
    local spline_elongation = 0.5       -- The default scalar elongation factor for the spline calculation
    local spline_types = {              -- The possible spline types. The default is the first the index
        "Cubic",     -- type:0          -- to the value that will be sent to the controller, starting at 0
        "Quintic"    -- type:1
    }

    local pathIntParams = { path_size, 0, 0 }
    local pathColor = { path_color_r, path_color_g, path_color_b, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

    simUI.setEditValue(ui_3, UI_IDs.ADVANCED.ELONGATION, tostring(spline_elongation))

    for i=0,#spline_types-1 do
        simUI.insertComboboxItem(ui_3, UI_IDs.ADVANCED.TYPE, i, spline_types[i+1])
    end

    return sim.createPath(-1, pathIntParams, nullptr, pathColor)
end

function lin_init()
    -- Default values
    local path_size = 5                 -- The line thickness of the path
    local path_color_r = 0              -- The red   component of the paths color
    local path_color_g = 1              -- The green component of the paths color
    local path_color_b = 0              -- The blue  component of the paths color

    local pathIntParams = { path_size, 0, 0 }
    local pathColor = { path_color_r, path_color_g, path_color_b, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

    return sim.createPath(-1, pathIntParams, nullptr, pathColor)
end

--[[
This functions updates the path for the spline functionality. It will start at the
current position and connects the entered points.
Input:
    handle      = Handle of the path that should be updated
    values      = Points for the path, the first point however will be the current position
--]]
function update_spline_path(handle, values)
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
    for _=1,8 do
        table.insert(data, 0)
    end

    -- Fill the table with the missing values
    for i=1,#values do
        table.insert(data, values[i][1])
        table.insert(data, values[i][2])
        table.insert(data, values[i][3])
        for _=1,8 do
            table.insert(data, 0)
        end
    end

    sim.cutPathCtrlPoints(handle, -1, 0)
    sim.insertPathCtrlPoints(handle, 0, 0, #values+1, data)
end

--[[
This function can show or hide the spline path.
Input:
    should_hide = true:  Any path is invisible
                  false: The crude path is visible
--]]
function spline_path_hide(should_hide)
    if (should_hide) then
        sim.cutPathCtrlPoints(spline_path_handle, -1, 0)
        for i=1,#loop_path_handles do
            sim.cutPathCtrlPoints(loop_path_handles[i], -1, 0)
        end
    else
        update_spline_path(spline_path_handle, spline_points_raw)
    end
end

--[[
This function can hide the lin path.
Input:
    should_hide = true:   The path is invisible
                  false:  Nothing happens, since the path is only shown when calculate is pressed
--]]
function lin_path_hide(should_hide)
    if (should_hide) then
        sim.cutPathCtrlPoints(lin_path_handle, -1, 0)
        for i=1,#loop_path_handles do
            sim.cutPathCtrlPoints(loop_path_handles[i], -1, 0)
        end
    end
end

--[[
This function gets called by the controller when the correct spline has been calculated. It
will show this new spline and destroy the previous temporary spline.
--]]
function show_calculated_path()

    local ret1 = sim.getStringSignal("path_general")
    local ret2 = sim.getStringSignal("path_loops")

    if not ret1 or not ret2 then
        return {},{},{},''
    end

    local obj1, _, err1 = json.decode(ret1, 1, nil)
    if err1 then
        sim.addStatusbarMessage("Error:", err1)
        return {},{},{},''
    end

    local obj2, _, err2 = json.decode(ret2, 1, nil)
    if err2 then
        sim.addStatusbarMessage("Error:", err2)
        return {},{},{},''
    end

    error_clear()

    -- There is no path possible
    if obj1.data == nil then
        error_append("There is no result possible for the entered values!")
        return {},{},{},''
    end

    -- The path could not be completed
    local last_config = obj1.data[#obj1.data]
    obj1.data[#obj1.data] = nil
    if last_config.m_x == 0 and
            last_config.m_y == 0 and
            last_config.m_z == 0 and
            last_config.m_a == 0 and
            last_config.m_b == 0 and
            last_config.m_c == 0 then
       error_append("There is no path available with a constant orientation. The Path got aborted!")
    end

    local tmp = {}
    local count = #obj1.data
    for i=1,count do
        table.insert(tmp, obj1.data[i].m_x)
        table.insert(tmp, obj1.data[i].m_y)
        table.insert(tmp, obj1.data[i].m_z)
        for _=1,8 do
            table.insert(tmp, 0)
        end
    end
    if simUI.getRadiobuttonValue(ui_1, 1022)==1 then
        sim.cutPathCtrlPoints(spline_path_handle, -1, 0)
        sim.insertPathCtrlPoints(spline_path_handle, 0, 0, count, tmp)
    else
        sim.cutPathCtrlPoints(lin_path_handle, -1, 0)
        sim.insertPathCtrlPoints(lin_path_handle, 0, 0, count, tmp)
    end

    -- Delete all loop paths
    for i=1,#loop_path_handles do
        sim.cutPathCtrlPoints(loop_path_handles[i], -1, 0)
    end

    -- Extract the loop values, create the paths and show them
    if obj2.data ~= nil then
        local pathIntParams = { 6, 0, 0 }
        local pathColor = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

        local loop_count = #obj2.data
        for i=1,loop_count do
            local point_count = #obj2.data[i]
            local final_points = {}
            for j=1,point_count do
                table.insert(final_points, obj2.data[i][j].m_x)
                table.insert(final_points, obj2.data[i][j].m_y)
                table.insert(final_points, obj2.data[i][j].m_z)
                for _=1,8 do
                    table.insert(final_points, 0)
                end
            end
            if loop_path_handles[i] ~= nil then
                sim.cutPathCtrlPoints(loop_path_handles[i], -1, 0)
            else
                loop_path_handles[i] = sim.createPath(-1, pathIntParams, nullptr, pathColor)
            end
            sim.insertPathCtrlPoints(loop_path_handles[i], 0, 0, point_count, final_points)
        end
        error_append("No continuous motion possible. Problematic areas are marked in red!")
    end

    if not error_is_empty() then
        error_show()
    end

    return {},{},{},''
end

---------------------------------------------------------------

--------------------Error UI functionality--------------------

function error_init()
    error_texts = {}
end

function ui_error_close(ui, id)
    if id ~= UI_IDs.ERROR.OK then
        return
    end
    error_clear()
    simUI.hide(ui)
end

function error_clear()
    for i=1,#error_texts do
        error_texts[i] = nil
    end
end

function error_append(text)
    table.insert(error_texts, text)
end

function error_set(text)
    error_clear()
    error_append(text)
end

function error_show()
    local final_text = ""
    for i=1,#error_texts do
        final_text = final_text .. error_texts[i] .. "<br><br>"

    end
    simUI.setText(ui_2, UI_IDs.ERROR.TEXT_BROWSER, final_text)
    simUI.show(ui_2)
end

function error_is_empty()
    return #error_texts == 0
end

--------------------------------------------------------------


--Close the UI
function closeEventHandler(h)
    simUI.hide(h)
end

--Close the second UI by hitting the Button
function buttonok(_,_)
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

--[[
Splits the given string at the given separator and returns the substrings as a table.
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
This function calculates the euler angles according to our convention (Roll, Pitch, Yaw)
Input:
    handle      = handle of the object
    id          = objective to calculate orientation relative to, or -1 for absolut
--]]
function get_orientation(handle, relative)
    mat = sim.getObjectMatrix(handle, relative)
    local epsilon = 0.00174532925            -- 0.1 degrees
    local phi, theta, psi
    if (math.abs(mat[1]) < epsilon and math.abs(mat[5]) < epsilon) then
        if math.abs(mat[9]-(-1)) < epsilon then
            phi = -math.atan2(-mat[7], mat[6])
        else
            phi = math.atan2(-mat[7], mat[6])
        end
        theta = -mat[9] * math.pi * 0.5
        psi = 0
    else
        phi = math.atan2(mat[5], mat[1]);
        theta = math.atan2(-mat[9], math.sqrt(math.pow(mat[10], 2) + math.pow(mat[11], 2)));
        psi = math.atan2(mat[10], mat[11]);
    end
    return {phi, theta, psi}
end

function set_orientation(handle, relative, angles)
    mat = sim.buildIdentityMatrix()
    mat = sim.rotateAroundAxis(mat, {1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, angles[3]) -- um x mit psi
    mat = sim.rotateAroundAxis(mat, {0, 1, 0}, {0, 0, 0}, angles[2])-- um y mit theta
    mat = sim.rotateAroundAxis(mat, {0, 0, 1}, {0, 0, 0}, angles[1])-- um z mit phi
    new_angles = sim.getEulerAnglesFromMatrix(mat)
    sim.setObjectOrientation(handle, relative, new_angles)
end

--Initialize the UI
if (sim_call_type==sim.syscb_init) then

    UI_IDs = {
        -- The IDs for the spline portion of the general UI
        SPLINE = {
            GROUP = 1020,
            COMBOBOX = 3000,
            X = 3001,
            Y = 3002,
            Z = 3003,
            CANCEL = 3004,
            DELETE = 3005,
            APPLY = 3006,
            INSERT = 3007,
            ADVANCED = 3008
        },
        -- The IDs for the Spline.Advanced window
        ADVANCED = {
            IMPORT_CB = 5000,
            IMPORT_BT = 5001,
            EXPORT_ED = 5002,
            EXPORT_BT = 5003,
            TYPE = 5005,
            ELONGATION = 5006,
            OK = 5004
        },
        -- The IDS of the error window
        ERROR = {
            TEXT_BROWSER = 6000,
            OK = 6001
        }
    }

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
            <combobox id="3000" on-change="ui_spline_cancel"></combobox>
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
                <button text="Cancel" id="3004" onclick="ui_spline_cancel"></button>
                <button text="Delete" id="3005" onclick="ui_spline_delete"></button>
                <br />
                <button text="Apply" id="3006" onclick="ui_spline_apply"></button>
                <button text="Insert" id="3007" onclick="ui_spline_insert"></button>
            </group>
            <button text="Advanced" id="3008" onclick="ui_spline_advanced"></button>


        </group>
        <button text="Calculate and Move" id="1011" enabled="false" onclick="CalculateIK"></button>
    </group>
</ui>]]

    local error = [[<ui closeable="false" on-close="ui_error_close" layout="vbox" title="Error">
	<text-browser id="6000"></text-browser>
	<button text="OK" on-click="ui_error_close" id="6001"></button>
</ui>]]



    splineIO = [[<ui closeable="false" on-close="ui_advanced_ok" layout="vbox" title="CSV functionality">
    <group layout="vbox">
        <label text="You can either import spline points from a vsc file
or export the current points to a file."></label>
        <group layout="hbox">
            <group layout="vbox">
                <label text="Import"></label>
                <combobox id="5000"></combobox>
                <button text="Import" onclick="ui_advanced_import" id="5001"></button>
            </group>
            <group layout="vbox">
                <label text="Export"></label>
                <edit id="5002" value=""></edit>
                <button text="Export" onclick="ui_advanced_export" id="5003"></button>
            </group>
        </group>
    </group>
    <group layout="vbox">
        <label text="You can select a spline type."></label>
        <combobox id="5005"></combobox>
    </group>
    <group layout="vbox">
        <label text="Choose the scalar elongation factor for the spline movement."></label>
        <edit id="5006" value=""></edit>
    </group>
    <button text="OK" onclick="ui_advanced_ok" id="5004"></button>
</ui>]]

    edit_ids = {}
    editvalues = {}
    jpedit_ids = {}
    jpeditvalues = {}
    c = {}
    cconf={}
    degree = false

    json=require("dkjson")

    ui_1=simUI.create(coord_dialog)
    ui_2=simUI.create(error)
    ui_3=simUI.create(splineIO)

    local myuis = {ui_1,ui_2,ui_3,ui_4,ui_5}
    sim.setStringSignal("uisignal",sim.packTable(myuis))

    ----------Added variables---------
    movement_allowed = false    -- Whether apply has been pressed once
    -- and the normal movement is allowed

    error_init()

    spline_points_raw  = {}
    spline_path_handle = spline_init()
    lin_path_handle    = lin_init()
    loop_path_handles  = {}
    ----------------------------------

    ----------Added initialization---------
    simUI.setEditValue(ui_1, 3020, tostring(1.0))                   -- Default velocity
    simUI.setEditValue(ui_1, 3021, tostring(1.0))                   -- Default acceleration
    ---------------------------------------

    ik_dummy = sim.getObjectHandle('ik_target')
    ik_target = sim.getObjectHandle('testTarget1')
    --ik_test = sim.getObjectHandle('testTarget2')
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
end