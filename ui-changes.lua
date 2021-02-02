-- In the coord_dialog script, you need to convert degrees to radians for IK,
-- as this controller has no way to determine which unit is used.
        -- Add after line 229:
        if (simUI.getRadiobuttonValue(ui,2007)==1) then
            tar_c[i] = math.rad(tonumber(tar_c[i]))
        end

-- In the coord_dialog script, some values are converted to radians twice.
-- Remove "math.rad" in lines 71, 72 & 73.

-- To add the splines UI:
-- TODO: