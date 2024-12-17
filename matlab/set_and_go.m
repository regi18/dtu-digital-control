function varargout = set_and_go(vel_ref, heading_ref_deg, distance)

if strcmp(heading_ref_deg, 'custom')
    heading_ref = -999999;
else
    heading_ref = deg2rad(heading_ref_deg);
end

assignin('base', 'heading_ref', heading_ref);
assignin('base', 'vel_ref', vel_ref);

time = distance / vel_ref;

out = sim('final_robot_model.slx', time);

dist = out.pose.Data(:, 1);
theta = out.pose.Data(:, 2);
x = out.pose.Data(:, 3);
y = out.pose.Data(:, 4);


% Determine the output based on the number of requested outputs
if nargout == 1
    varargout{1} = out;
elseif nargout == 2
    varargout = {x, y};
elseif nargout == 4
    varargout = {dist, theta, x, y};
else
    error('Invalid number of output arguments. Use either 1 or 4 outputs.');
end

end