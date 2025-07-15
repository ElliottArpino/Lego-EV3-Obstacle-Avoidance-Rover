%% Part 1: Apriltag Recognition
clear; close all; clc
vid_rec.vid_type = 'winvideo';
vid_rec.src_num = 2;
vid_rec.img_format = 'YUY2_1280x720';

calib = load('cameraParams.mat');
cam_calib = calib.cameraParams;
lv = labvision(vid_rec, cam_calib);

at_origin    = apriltag_obj(50, 'vehicle',   [-80 80 -80 80], 108);
at_vehicle   = apriltag_obj(51, 'obstacle',  [-80 80 -80 80], 108);
at_obstacle1 = apriltag_obj(52, 'origin',    [], 108);
at_obstacle2 = apriltag_obj(53, 'obstacle',  [-80 80 -80 80], 108);

lv.save_tag_obj(at_origin);
lv.save_tag_obj(at_vehicle);
lv.save_tag_obj(at_obstacle1);
lv.save_tag_obj(at_obstacle2);
lv.capture_scene;

scene = lv.scene;
K = lv.calib.Intrinsics.K;

T_CT = lv.origin_tag.T_CT;
XY = K * T_CT(1:3,4);
UV = [XY(1:2)/XY(3) scene(1).center]

T_CT = lv.vehicle_tag.T_CT;
XY = K * T_CT(1:3,4);
UV = [XY(1:2)/XY(3) scene([scene.id]==51).center]

for k = 1:numel(lv.obstacle_tags)
    T_CT = lv.obstacle_tags(k).T_CT;
    XY = K * T_CT(1:3,4);
    UV = [XY(1:2)/XY(3) scene([scene.id]==lv.obstacle_tags(k).id).center]
end

disp('Origin T_0T'); disp(lv.origin_tag.T_0T)
disp('Origin T_CT'); disp(lv.origin_tag.T_CT)
disp('Vehicle T_0T'); disp(lv.vehicle_tag.T_0T)
disp('Vehicle T_CT'); disp(lv.vehicle_tag.T_CT)
disp('Obstacle 1 T_0T'); disp(lv.obstacle_tags(1).T_0T)
disp('Obstacle 1 T_CT'); disp(lv.obstacle_tags(1).T_CT)
disp('Obstacle 2 T_0T'); disp(lv.obstacle_tags(2).T_0T)
disp('Obstacle 2 T_CT'); disp(lv.obstacle_tags(2).T_CT)

T = lv.vehicle_tag.T_0T;
position = T(1:3,4);
heading = atan2(T(2,1), T(1,1));
fprintf('Vehicle Position (x, y, z): [%.2f, %.2f, %.2f]\n', position(1), position(2), position(3));
fprintf('Vehicle Heading (yaw): %.2f radians\n', heading);

%% Part 2: Occupancy Map
%% Obstacle Polygons – Scene 1 (Demo)
% Scene 1
obs1X = ([463.08, 468.932, 668.672, 662.82] / 1000) + 0.1;  
obs1Y = ([448.127, 258.415, 248.213, 458.329] / 1000) + 0.05;
obs2X = ([378.01, 567.065, 578.01, 367.065] / 1000) - 0.1;   
obs2Y = ([-15.0181, -15.1024, -214.718, -214.803] / 1000) - 0.1;

%% Obstacle Polygons – Scene 2 
% Scene 2
obs1X = ([463.08, 468.932, 668.672, 662.82] / 1000) + 0.1;  
obs1Y = ([448.127, 258.415, 248.213, 458.329] / 1000) + 0.05;
obs2X = ([378.01, 567.065, 578.01, 367.065] / 1000) - 0.0; 
obs2Y = ([-15.0181, -15.1024, -214.718, -214.803] / 1000) + 0.2;

%% Obstacle Polygons – Scene 3 
% Scene 3
obs1X = ([463.08, 468.932, 668.672, 662.82] / 1000) - 0.05; 
obs1Y = ([448.127, 258.415, 248.213, 458.329] / 1000) - 0.05;
obs2X = ([378.01, 567.065, 578.01, 367.065] / 1000) + 0.05;  
obs2Y = ([-15.0181, -15.1024, -214.718, -214.803] / 1000) + 0.05;

%%
resolution = 400;                    % Grid resolution: 400 cells per meter (~2.5 mm per cell)
xRange_m = -0.3:1/resolution:1.1;
yRange_m = -0.5:1/resolution:0.9;
[Xm, Ym] = meshgrid(xRange_m, yRange_m);
inObs1 = inpolygon(Xm, Ym, obs1X, obs1Y);
inObs2 = inpolygon(Xm, Ym, obs2X, obs2Y);
occupied = inObs1 | inObs2;
occupied_flip = flipud(occupied);
map = occupancyMap(occupied_flip, resolution);
cellSize = xRange_m(2) - xRange_m(1);
map.GridLocationInWorld = [min(xRange_m)+cellSize/2, min(yRange_m)+cellSize/2];

figure;
imshow(1 - occupied_flip, 'XData', xRange_m, 'YData', yRange_m);
set(gca, 'YDir', 'reverse');
axis equal; axis tight; axis on;
xlabel('X [m]'); ylabel('Y [m]');

%% Part 2a: Potential Field Computation
goalPos = [1, -0.2];          % IMPORTANT: goalPos defines the desired goal position [m]
K = [1, 1e-5, 5e-6 , 1];       % IMPORTANT: K = [k_att, k_rep, k_rot, f_max]; set f_max > 0 to enable clamping
r0_meters = 1;             % IMPORTANT: r0_meters is the obstacle influence radius in [m]
r0 = round(r0_meters * resolution);

[fx, fy] = gradient_path(occupied_flip, goalPos, K, r0, xRange_m, yRange_m, false);

figure;
imshow(1 - occupied_flip, 'XData', xRange_m, 'YData', yRange_m);
set(gca, 'YDir', 'reverse');
axis equal; axis tight; axis on;
hold on;
[Xg, Yg] = meshgrid(xRange_m, yRange_m);
ds = 5;
scaleFactor = 0.009;      % IMPORTANT: scaleFactor scales the force vectors for visualization
quiver(Xg(1:ds:end, 1:ds:end), Yg(1:ds:end, 1:ds:end), ...
       fx(1:ds:end, 1:ds:end)*scaleFactor, fy(1:ds:end, 1:ds:end)*scaleFactor, 0, 'r');
xlabel('X [m]'); ylabel('Y [m]');
hold off;

%% Part 2b: A* Path Planning
startPos = [0, 0.7];
gridIndStart = world2grid(map, startPos);
gridIndGoal  = world2grid(map, goalPos);
gridIndStart(1) = map.GridSize(1) - gridIndStart(1) + 1;
gridIndGoal(1)  = map.GridSize(1) - gridIndGoal(1) + 1;
planner_obj = plannerAStarGrid(map);
pathAStar = plan(planner_obj, [gridIndStart(1), gridIndStart(2)], [gridIndGoal(1), gridIndGoal(2)]);
flippedPath = pathAStar;
flippedPath(:,1) = map.GridSize(1) - pathAStar(:,1) + 1;
worldCoordinates_AStar = grid2world(map, flippedPath);

figure;
imshow(1 - occupied_flip, 'XData', xRange_m, 'YData', yRange_m);
set(gca, 'YDir', 'reverse');
axis equal; axis tight; axis on;
hold on;
plot(worldCoordinates_AStar(:,1), worldCoordinates_AStar(:,2), 'r.-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
hold off;

%% Part 2c: Potential Field Path
stepSize = 0.001;    % Step size for gradient descent [m]
maxSteps = 10000;
potentialPath = followGradient(fx, fy, xRange_m, yRange_m, startPos, goalPos, stepSize, maxSteps);

figure;
imshow(1 - occupied_flip, 'XData', xRange_m, 'YData', yRange_m);
set(gca, 'YDir', 'reverse');
axis equal; axis tight; axis on;
hold on;
plot(potentialPath(:,1), potentialPath(:,2), 'c.-', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
hold off;

%% Part 2d: Sparsify Potential Path
segmentThreshold = 0.1;  % IMPORTANT: segmentThreshold is the minimum distance between path points to keep [m]
sparsePath = potentialPath(1,:);
for i = 2:size(potentialPath,1)
    if norm(potentialPath(i,:) - sparsePath(end,:)) >= segmentThreshold
        sparsePath = [sparsePath; potentialPath(i,:)];
    end
end
if norm(sparsePath(end,:) - potentialPath(end,:)) > 1e-3
    sparsePath = [sparsePath; potentialPath(end,:)];
end

figure;
imshow(1 - occupied_flip, 'XData', xRange_m, 'YData', yRange_m);
set(gca, 'YDir', 'reverse');
axis equal; axis tight; axis on;
hold on;
plot(potentialPath(:,1), potentialPath(:,2), 'c.-', 'LineWidth', 1);
plot(sparsePath(:,1), sparsePath(:,2), 'bo-', 'LineWidth', 2, 'MarkerSize', 6);
xlabel('X [m]'); ylabel('Y [m]');
hold off;

%% Part 3: Robot Integration

%% Part 3a: Controller Settings
% IMPORTANT: EV3 CONNECTIONS - EV3 board via USB; left motor port 'A', right motor port 'D'
myev3 = legoev3('usb');
motorLeft  = motor(myev3, 'A');
motorRight = motor(myev3, 'D');

R = 2.8;        % Wheel radius in cm
L = 12.0;       % Distance between wheels in cm

v_forward_base = 50;          % Base forward speed [cm/s]
maxForwardSpeed_cmps = 30;    % Maximum allowed forward speed [cm/s]
velScaleFactor = 0.8;         % Additional speed scaling (0 < factor <= 1)
maxWheelSpeed_degPerSec = 300; % Maximum motor speed [deg/s] for safety

Kp_heading = 1.5;             % PD controller proportional gain
Kd_heading = 1;               % PD controller derivative gain
headingErrorThreshold = deg2rad(2);   % Threshold for heading alignment (2° in radians)
dt = 0.01;                    % Controller loop time [s]

initial_heading = 0;          % Set initial heading (simulation/starting condition)
current_heading = initial_heading;

fprintf('Starting open-loop execution with improved heading PD control along the scaled potential path...\n');

% IMPORTANT: Scaling the sparse path and converting from meters to centimeters
pathScaleFactor = 0.1;        % Scale factor applied to the potential field path
path_cm = (sparsePath * pathScaleFactor) * 100;

%% Part 3b: Open-Loop PD control along Potential Path
numPoints = size(path_cm, 1);
for i = 1:numPoints-1
    current_wp = path_cm(i, :);
    next_wp = path_cm(i+1, :);
    delta = next_wp - current_wp;
    segmentDistance = norm(delta);
    thetaRef = atan2(delta(2), delta(1));
    prevError = 0;
    while true
        headingError = thetaRef - current_heading;
        headingError = atan2(sin(headingError), cos(headingError));
        if abs(headingError) < headingErrorThreshold
            break;
        end
        dError = (headingError - prevError) / dt;
        omegaCmd = Kp_heading * headingError + Kd_heading * dError;
        prevError = headingError;
        if omegaCmd > 0
            v_left = - (L/2)*abs(omegaCmd);
            v_right = (L/2)*abs(omegaCmd);
        else
            v_left = (L/2)*abs(omegaCmd);
            v_right = - (L/2)*abs(omegaCmd);
        end
        omega_left = v_left / R;
        omega_right = v_right / R;
        degSec_left = rad2deg(omega_left);
        degSec_right = rad2deg(omega_right);
        degSec_left = max(min(degSec_left, maxWheelSpeed_degPerSec), -maxWheelSpeed_degPerSec);
        degSec_right = max(min(degSec_right, maxWheelSpeed_degPerSec), -maxWheelSpeed_degPerSec);
        motorLeft.Speed = degSec_left;
        motorRight.Speed = degSec_right;
        start(motorLeft); start(motorRight);
        pause(dt);
        stop(motorLeft); stop(motorRight);
        current_heading = current_heading + omegaCmd * dt;
        current_heading = atan2(sin(current_heading), cos(current_heading));
        fprintf('Turning: Current Heading = %.1f°, Target = %.1f°, Error = %.1f°\n',...
            rad2deg(current_heading), rad2deg(thetaRef), rad2deg(headingError));
    end
    fprintf('Segment %d: Aligned to desired heading (%.1f°).\n', i, rad2deg(thetaRef));
    v_forward = v_forward_base;
    if v_forward > maxForwardSpeed_cmps
        v_forward = maxForwardSpeed_cmps;
    end
    v_forward = v_forward * velScaleFactor;
    t_forward = segmentDistance / v_forward;
    omega_forward = v_forward / R;
    degSec_forward = rad2deg(omega_forward);
    degSec_forward = max(min(degSec_forward, maxWheelSpeed_degPerSec), -maxWheelSpeed_degPerSec);
    fprintf('Segment %d: Driving forward %.2f cm over %.2f s (v_forward = %.2f cm/s)...\n',...
        i, segmentDistance, t_forward, v_forward);
    motorLeft.Speed = degSec_forward;
    motorRight.Speed = degSec_forward;
    start(motorLeft); start(motorRight);
    pause(t_forward);
    stop(motorLeft); stop(motorRight);
    pause(0.1);
end
stop(motorLeft); stop(motorRight);
fprintf('Open-loop execution with improved heading PD control complete.\n');

%% Part 4: Functions
function [fx, fy] = gradient_path(obs_map, goal, K, r0, x, y, plot_map)
    if nargin < 7, plot_map = 0; end
    k_att = K(1); 
    k_rep = K(2); 
    k_rot = K(3); 
    f_max = K(4);
    
    [X, Y] = meshgrid(x, y);
    sz = size(obs_map);
    fx = -k_att * (X - goal(1));  % Attractive force in x-direction
    fy = -k_att * (Y - goal(2));  % Attractive force in y-direction
    F_mag = sqrt(fx.^2 + fy.^2);
    
    % IMPORTANT: Apply clamping on attractive force only if f_max > 0
    if f_max > 0
        exceed_idx = F_mag > f_max;
        fx(exceed_idx) = fx(exceed_idx) .* (f_max ./ F_mag(exceed_idx));
        fy(exceed_idx) = fy(exceed_idx) .* (f_max ./ F_mag(exceed_idx));
    end
    
    obs_perim = bwperim(obs_map);
    perim_idx = find(obs_perim(:));
    n_perim = length(perim_idx);
    [pyIdx, pxIdx] = ind2sub(sz, perim_idx);
    repulse_idx = -r0:r0;
    [rx, ry] = meshgrid(repulse_idx);
    cellSize = x(2) - x(1);
    rx = rx * cellSize;
    ry = ry * cellSize;
    r = sqrt(rx.^2 + ry.^2);
    mask = (r <= (r0 * cellSize)) & (r > 0);
    f_rep_x = zeros(size(r));
    f_rep_y = zeros(size(r));
    f_rot_x = zeros(size(r));
    f_rot_y = zeros(size(r));
    f_rep_x(mask) = k_rep * (1./(r(mask).^2)) .* (rx(mask)./r(mask));
    f_rep_y(mask) = k_rep * (1./(r(mask).^2)) .* (ry(mask)./r(mask));
    f_rot_x(mask) = k_rot * (-ry(mask)) ./ (r(mask).^3);
    f_rot_y(mask) = k_rot * (rx(mask)) ./ (r(mask).^3);
    repulse_map_x = f_rep_x + f_rot_x;
    repulse_map_y = f_rep_y + f_rot_y;
    Nx = length(x); Ny = length(y);
    for k = 1:n_perim
        r_center = pyIdx(k); 
        c_center = pxIdx(k);
        r_dest_start = max(r_center - r0, 1);
        r_dest_end = min(r_center + r0, Ny);
        c_dest_start = max(c_center - r0, 1);
        c_dest_end = min(c_center + r0, Nx);
        src_r_start = 1 + (r_dest_start - (r_center - r0));
        src_r_end = (2*r0 + 1) - ((r_center + r0) - r_dest_end);
        src_c_start = 1 + (c_dest_start - (c_center - r0));
        src_c_end = (2*r0 + 1) - ((c_center + r0) - c_dest_end);
        fx(r_dest_start:r_dest_end, c_dest_start:c_dest_end) = ...
            fx(r_dest_start:r_dest_end, c_dest_start:c_dest_end) + ...
            repulse_map_x(src_r_start:src_r_end, src_c_start:src_c_end);
        fy(r_dest_start:r_dest_end, c_dest_start:c_dest_end) = ...
            fy(r_dest_start:r_dest_end, c_dest_start:c_dest_end) + ...
            repulse_map_y(src_r_start:src_r_end, src_c_start:src_c_end);
    end
    fx(obs_map > 0) = 0;
    fy(obs_map > 0) = 0;
    if plot_map
        figure;
        imshow(obs_map, 'XData', x, 'YData', y);
        set(gca, 'YDir', 'normal'); 
        axis equal; axis tight; axis on; 
        hold on;
        ds = 5; 
        scaleFactor = 0.005;
        quiver(X(1:ds:end,1:ds:end), Y(1:ds:end,1:ds:end),...
            fx(1:ds:end,1:ds:end)*scaleFactor, fy(1:ds:end,1:ds:end)*scaleFactor, 0, 'r');
        xlabel('X [m]'); ylabel('Y [m]'); 
        hold off;
    end
end

function path = followGradient(fx, fy, x, y, startPos, goalPos, stepSize, maxSteps)
    distThreshold = 0.01;
    currentPos = startPos;
    path = currentPos;
    for i = 1:maxSteps
        dx = interp2(x, y, fx, currentPos(1), currentPos(2), 'linear', 0);
        dy = interp2(x, y, fy, currentPos(1), currentPos(2), 'linear', 0);
        gradVec = [dx, dy];
        magGrad = norm(gradVec);
        if magGrad < 1e-9
            directVec = goalPos - currentPos;
            if norm(directVec) > 1e-9
                direction = directVec / norm(directVec);
            else
                warning('Goal reached or very close.');
                break;
            end
        else
            direction = gradVec / magGrad;
        end
        currentPos = currentPos + stepSize * direction;
        path = [path; currentPos];
        if norm(currentPos - goalPos) < distThreshold
            disp('Reached the goal region!');
            break;
        end
    end
    if norm(currentPos - goalPos) >= distThreshold
        disp('Potential flow path did not converge exactly; appending goal.');
        path = [path; goalPos];
    end
end
