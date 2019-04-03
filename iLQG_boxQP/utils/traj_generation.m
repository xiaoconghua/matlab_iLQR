function refTraj = traj_generation( )
%TRAJ_GENERATION Summary of this function goes here
%   Detailed explanation goes here
addpath('utils');

kCircleRadius = 10;
kFinalAngle = pi/2;
kRefSize    = 50;
kTimestep   = 0.05;

kIncrementalAngle = kFinalAngle / (kRefSize - 1);

refTrajMsg  = cell(kRefSize, 1);
refCurvMat  = zeros(kRefSize, 1);
refVelMat   = zeros(kRefSize, 1);
refArcLenMat= zeros(kRefSize, 1);
refPosMat   = zeros(kRefSize, 2);
refLatBoundsMat = zeros(kRefSize, 2);
refVelBoundsMat = zeros(kRefSize, 2);

prev_position = [0, 0];
for i = 1 : kRefSize
    waypoint_i = waypoint();
    waypoint_i.time_along_traj = i * kTimestep;
    rotate_angle = kIncrementalAngle*(i - 1);
    waypoint_i.position = [kCircleRadius*cos(rotate_angle), kCircleRadius*sin(rotate_angle)];
    waypoint_i.curvature = 1/kCircleRadius;
    if i == 1
        waypoint_i.arclength = 0;
        dist_diff = 0;
    else
        dist_diff = norm(waypoint_i.position - prev_position);
        waypoint_i.arclength = refTrajMsg{i - 1}.arclength + dist_diff;
    end
    
    waypoint_i.velocity = dist_diff / kTimestep;
    
    refTrajMsg(i) = {waypoint_i};
    
    prev_position = waypoint_i.position;
    
    refPosMat(i, :)     = waypoint_i.position;
    refCurvMat(i, :)    = waypoint_i.curvature;
    refVelMat(i, :)     = waypoint_i.velocity;
    refArcLenMat(i, :)    = waypoint_i.arclength;
    refLatBoundsMat(i, :) = waypoint_i.lateral_bounds;
    refVelBoundsMat(i, :) = waypoint_i.velocity_bounds;
end


refTraj.waypoints = refTrajMsg;
refTraj.curvature = refCurvMat;
refTraj.velocity  = refVelMat;
refTraj.position  = refPosMat;
refTraj.arclength = refArcLenMat;
refTraj.latbounds = refLatBoundsMat;
refTraj.velbounds = refVelBoundsMat;
refTraj.num_nodes = kRefSize;
%% plot the trajectory
figure;
subplot(3, 1, 1)
plot(refPosMat(:, 1), refPosMat(:, 2))
legend('position')
subplot(3, 1, 2)
plot(refArcLenMat)
legend('arclength')
subplot(3, 1, 3)
plot(refVelMat)
legend('vel')
end

