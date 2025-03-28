clear all;
clc;
close all;

% LOAD ROBOT
robot = loadrobot("kinovaGen3");
robot.Gravity = [0 0 -9.81];
robot.DataFormat = 'column';

% INIT SETUP
q_initial = [0 15 180 -130 0 55 90]' * pi/180;  % Convert to radians
eeName = string(robot.BodyNames(length(robot.BodyNames)));
T_home = getTransform(robot, q_initial, eeName);
toolPositionHome = T_home(1:3, 4);

% WAYPOINTS AND ORIENTATIONS
points = [0.0 0.0 0.0;
          -0.1 0.2 0.2;
          -0.2 0.0 0.1;
          -0.1 -0.2 0.2;
           0.0 0.0 0.0]';
       
orientations = [pi/2    0   pi/2;
               (pi/2)+(pi/4)      0   pi/2;
                pi/2    0   pi;
               (pi/2)-(pi/4)       0    pi/2;
                pi/2    0   pi]';

TimeToGo = [0 5 15 20 25];

% RANDOM WAYPOINT AND DUPLICATE
random_index = randi([2, size(points, 2) - 1]);
points_ = [points(:, 1:random_index), points(:, random_index:end)];
orientations_ = [orientations(:, 1:random_index), orientations(:, random_index:end)];

fprintf('Waypoint selected (random index): %d\n', random_index);
disp(points(:, random_index));
fprintf('Waypoint list:\n');
disp(points_);
waypoints = toolPositionHome + points_;

% RANDOM STOPPING TIME
RandomStoppingTime = randi([4, 10]);  % Random stop duration
TimeToGo = [TimeToGo(1:random_index), TimeToGo(random_index) + RandomStoppingTime, TimeToGo(random_index + 1:end) + RandomStoppingTime];
waypointTimes = 0.5 * TimeToGo;  % Adjusted stop times
fprintf('Random stopping time [s]: %d\n', RandomStoppingTime);
disp(waypointTimes);

% Discretized in steps
Number_of_steps = 20;
ts = waypointTimes(end) / Number_of_steps;  % Time step
trajTimes = 0:ts:waypointTimes(end);

% End-effector velocity at waypoints:
waypointVels = 0.8 * [0 1 0;
                     -1 0 0;
                      0 -1 0;
                      1 0 0;
                      0 1 0]';

waypointVels = [waypointVels(:, 1:random_index), zeros(3, 1), waypointVels(:, random_index + 1:end)];

% Trajectory interpolation
trajType = "pchip";  % Piecewise cubic Hermite interpolation
index = 1:size(waypoints, 2);
indexq = linspace(1, numel(index), numel(trajTimes));
InterpolatedPoints = zeros(size(waypoints, 1), numel(indexq));

for i = 1:size(waypoints, 1)
    v = waypoints(i, :);
    InterpolatedPoints(i, :) = interp1(index, v, indexq, trajType);
end

% Acceleration at waypoints (calculated from velocity)
waypointAccels = zeros(size(waypointVels));
waypointAccelTimes = diff(waypointTimes) / 4;

% Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = q_initial';

% Normalize angles between -pi and +pi
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

numJoints = numel(robot.homeConfiguration);
q = zeros(numJoints, size(InterpolatedPoints, 2));

for idx = 1:size(InterpolatedPoints, 2)
    % Check if the waypoint is reachable
    tgtPose = trvec2tform(InterpolatedPoints(:, idx)') * eul2tform([pi/2 0 pi/2]);
    
    [config, info] = ik(eeName, tgtPose, ikWeights', ikInitGuess');
    
    % Check if the solution is valid
    if info.Status == 'success'
        q(:, idx) = config';
    else
        % Handle unreachable waypoint
        warning('Waypoint %d is unreachable. Skipping.', idx);
        q(:, idx) = q(:, max(1, idx - 1));  % Use previous valid config
    end
end

qd = gradient(q, ts);
qdd = gradient(qd, ts);

% Plotting Mode
plotMode = 1;  % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
show(robot, q_initial, 'Frames', 'off', 'PreservePlot', false);
hold on;

if plotMode == 1
    hTraj = plot3(waypoints(1, 1), waypoints(2, 1), waypoints(3, 1), 'b.-');
end
plot3(waypoints(1, :), waypoints(2, :), waypoints(3, :), 'ro', 'LineWidth', 2);
axis auto;
view([30 15]);

for idx = 1:numel(trajTimes)
    config = q(:, idx)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(robot, config', eeName);
    
    % Plot trajectory
    if plotMode == 1
        eePos = tform2trvec(eeTform);
        set(hTraj, 'xdata', [hTraj.XData eePos(1)], ...
            'ydata', [hTraj.YData eePos(2)], ...
            'zdata', [hTraj.ZData eePos(3)]);
    elseif plotMode == 2
        plotTransforms(tform2trvec(eeTform), tform2quat(eeTform), 'FrameSize', 0.05);
    end
    
    % Show the robot
    show(robot, config', 'Frames', 'off', 'PreservePlot', false);
    title(['Trajectory at t = ' num2str(trajTimes(idx))]);
    drawnow;
end
