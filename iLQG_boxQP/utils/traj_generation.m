function refTraj = traj_generation( )
%TRAJ_GENERATION Summary of this function goes here
%   Detailed explanation goes here
addpath('utils');

kCircleRadius = 10;
kFinalAngle = pi/2;
kRefSize    = 10;
kTimestep   = 0.1;

kIncrementalAngle = kFinalAngle / (kRefSize - 1);

refTrajMsg  = cell(kRefSize, 1);
refCurvMat  = zeros(kRefSize, 1);
refVelMat   = zeros(kRefSize, 1);
refArcLenMat    = zeros(kRefSize, 1);
refPosMat   = zeros(kRefSize, 2);
refLatBoundsMat = zeros(kRefSize, 2);
refVelBoundsMat = zeros(kRefSize, 2);

prev_position = [0, 0];
for i = 1 : kRefSize
    waypoint_i = waypoint();
    waypoint_i.time_along_traj = i * kTimestep;
    waypoint_i.curvature = 0;
    rotate_angle = kIncrementalAngle*(i - 1);
    waypoint_i.position = [kCircleRadius*cos(rotate_angle), kCircleRadius*sin(rotate_angle)];
    if i == 1
        waypoint_i.arclength = 0;
    else
        waypoint_i.arclength = refTrajMsg{i - 1}.arclength + norm(waypoint_i.position - prev_position);
    end
    
    waypoint_i.velocity = waypoint_i.arclength / kTimestep;
    
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
plot(refPosMat(:, 1), refPosMat(:, 2))
hold on
plot(refArcLenMat)
end

