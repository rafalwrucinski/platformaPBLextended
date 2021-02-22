function helperVisualizeExecuteRobots(f, robotPoses, wayPoints, ranges, angles, startLocations, goalLocations, showPaths, xLim, yLim, robotMeshes,vizLim)

persistent map

maxRange = 10.2;
resolution = 10;

if isempty(map)
   map= occupancyMap((yLim(2)*20),(xLim(2)*20),resolution,'grid');
   map.GridOriginInLocal = [-xLim(2) -yLim(2)];
end

%% Initilize working variables
numRobots = size(robotPoses, 2);
% numLoad = size(goalLocations(end,1:2), 1);
numLoad=1;
numGoals = size(goalLocations(1:end-1,1:2), 1);
numStarts = size(startLocations, 1);

%% Extract data from inputs

% Construct translation from state
xyz = [robotPoses(1:2,:); zeros(1,numRobots)]; % 2-D so z is zero

% Create quaternion from rotation state
theta = robotPoses(3,:);


if ~isempty(ranges)
    characterPose = [robotPoses(1) robotPoses(2) robotPoses(3)];
    scans= lidarScan(ranges,angles);
    insertRay(map, characterPose, scans, maxRange);
end

%% Visualize Results
ax = f.CurrentAxes;
show(map,'Parent', ax);
light
hold(ax, 'on');

% plotTransforms([goalLocations(1:end-1,1:2), zeros(numGoals, 1)], eul2quat(zeros(numGoals, 3)), 'MeshFilePath', 'exampleWarehouseBlockExecuteTasks.stl', 'MeshColor', 'g', 'Parent', ax);
% plotTransforms([goalLocations(end,1:2), zeros(numLoad, 1)], eul2quat(zeros(numLoad, 3)), 'MeshFilePath', 'exampleWarehouseBlockExecuteTasks.stl', 'MeshColor', 'b', 'Parent', ax);
% plotTransforms([startLocations, zeros(numStarts, 1)], eul2quat(zeros(numStarts, 3)), 'MeshFilePath', 'exampleWarehouseBlockExecuteTasks.stl', 'MeshColor', 'k', 'Parent', ax);
% text(goalLocations(end, 1), goalLocations(end, 2), ones(numLoad, 1)* 2, 'Final Goal', 'FontSize', 12);
% text(goalLocations(1:end-1, 1), goalLocations(1:end-1, 2), ones(numGoals, 1)* 2, 'Goal', 'FontSize', 12);
% text(startLocations(:, 1), startLocations(:, 2), ones(numStarts, 1)* 2, 'Start Station', 'FontSize', 12);

plotTransforms([goalLocations(1:end-1,1:2), zeros(numGoals, 1)], eul2quat(zeros(numGoals, 3)), 'FrameSize', 0.5, 'MeshFilePath', 'model3D.stl', 'MeshColor', 'g', 'Parent', ax);
plotTransforms([goalLocations(end,1:2), zeros(numLoad, 1)], eul2quat(zeros(numLoad, 3)),'FrameSize' , 0.5, 'MeshFilePath', 'model3D.stl', 'MeshColor', 'b', 'Parent', ax);
plotTransforms([startLocations, zeros(numStarts, 1)], eul2quat(zeros(numStarts, 3)),'FrameSize',0.5, 'MeshFilePath', 'model3D.stl', 'MeshColor', 'k', 'Parent', ax);
text(goalLocations(end, 1), goalLocations(end, 2), ones(numLoad, 1), 'Final Goal', 'FontSize', 12);
text(goalLocations(1:end-1, 1), goalLocations(1:end-1, 2), ones(numGoals, 1), 'Goal', 'FontSize', 12);
text(startLocations(:, 1), startLocations(:, 2), ones(numStarts, 1), 'Start Station', 'FontSize', 12);

for robotIdx = 1:numRobots
    
    % Extract the information for the ith robot
    robotQuaternion = eul2quat([0 0 theta(robotIdx)], 'xyz');
    worldToRobotTForm = [axang2rotm([0 0 1 theta(robotIdx)]) xyz(:,robotIdx); 0 0 0 1];
    robotPathWaypoints = wayPoints(:,:,robotIdx);
    
    robotMesh = robotMeshes{1};
    meshTranslation = robotMeshes{2};
    robotTform = worldToRobotTForm*trvec2tform(meshTranslation);
    
    % Plot the robot
    plotTransforms(robotTform(1:3,4)', robotQuaternion, 'FrameSize', 0.5,'MeshFilePath', robotMesh, 'Parent', ax);
    
    % Plot the path
    if showPaths
        plot(robotPathWaypoints(:,1), robotPathWaypoints(:,2), 'x-');
    end
end

% Select the view
xlim([vizLim(4) vizLim(1)]);
ylim([vizLim(3) vizLim(2)]);
zlim([0 1]);

drawnow;
hold(ax, 'off');
end