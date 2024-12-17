function [vel_ref_new, heading, turnrate, useHeadingController] = get_ref(t, vel_ref, path)

heading = 0;
turnrate = 0;
vel_ref_new = vel_ref;

if t < 1.5
    heading = 0;
    useHeadingController = 1;
elseif t < 2.1
    heading = deg2rad(-48);
    useHeadingController = 1;
elseif t < 6.4
    useHeadingController = 0;
    turnrate = vel_ref / 0.5;
elseif t < 7.5
    useHeadingController = 1;
    heading = deg2rad(210);
else
    useHeadingController = 1;
    heading = deg2rad(182);
    vel_ref_new = vel_ref;
end

end




