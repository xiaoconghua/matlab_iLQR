function dx = kinodynamics(x, u, refTraj)
%KINODYNAMICS Specify a 4 states kinodynamics of a car.
% Inputs
% ======
% x = [arc_len, lat_offset, theta_err, vel] 
% u = [delta, accel]
% refTraj gives the reference trajectory of the problem. The reference
% trajectory has to be represented in path frame.

L = 4; % vehicle length

interp_waypoint = GetInterpWaypoint(x(1), refTraj);

dx = zeros(size(x));

curv = interp_waypoint.curvature;

dx(1) = x(4) / (1 - curv*x(2))*cos(x(3));
dx(2) = x(4) * sin(x(3));
dx(3) = x(4) * (1/L*tan(u(1)) - curv / (1 - curv*x(2)));
dx(4) = u(2);

end
