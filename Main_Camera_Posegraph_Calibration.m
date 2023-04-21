% This script will calibrate a multi-camera system by setting up a
% posegraph using estimated poses from the ROS node from images of the
% ArUco double-sided board.
%
% The ROS node should be run prior to running this calibration as this
% script requires the CSV file "camera_board_poses_timestamped.csv"
%
% The output optimised poses will be saved in a YAML file called
% "optimised_camera_poses.yaml"
% 
% Author: Jasprabhjit Mehami

clc;
close all;
clear;

%robotics toolbox
run(fullfile('ext_lib','rvctools', 'startup_rvc.m'));

%yaml reader package
addpath(genpath(fullfile('ext_lib','yamlmatlab-master')));

addpath('src');

%% Check directorties and read camera board poses


disp("Reading camera info YAML file and board poses CSV file ...");
calibrationDir = 'calibration_files';

cameraInfoYAML = fullfile(calibrationDir, 'camera_info.yaml');
if ~exist(cameraInfoYAML, "file")
    error("%s is missing", cameraInfoYAML)
end

cameraInfoStruct = yaml.ReadYaml(cameraInfoYAML);
if ~isfield(cameraInfoStruct, 'topics')
    error("Missing 'topics' parameter in %s", cameraInfoYAML)
end

cameraInfoStruct = yaml.ReadYaml(cameraInfoYAML);
if ~isfield(cameraInfoStruct, 'initial_extrinsic')
    error("Missing 'initial_extrinsic' parameter in %s", cameraInfoYAML)
end

cameraBoardCSV = fullfile(calibrationDir, 'camera_board_poses_timestamped.csv');
if ~exist(cameraBoardCSV, "file")
    error("%s is missing", cameraBoardCSV)
end

% Read as MATLAB table
dataCameraPoses = readtable(cameraBoardCSV);
cameraIDs = table2array(dataCameraPoses(:,1));
topicNames = string(table2cell(dataCameraPoses(:,end)));
poses = table2array(dataCameraPoses(:,3:end-1));
timeStamps = table2array(dataCameraPoses(:,2));

% get unique topic names and camera IDs
[cameraIDsUnique, ixdUnique] = unique(cameraIDs);
topicNamesUnique = topicNames(ixdUnique);

%check if atleast one pose for all camera topic has been found
for i = 1:length(cameraInfoStruct.topics)
    if ~any(strcmp(topicNamesUnique, cameraInfoStruct.topics(i)))
        error("Camera with topic name %s has no estimated board poses. Cannot solve posegraph. Please re-run the multi-camera calibration node to get all possible board poses or re-collect calibration data.")
    end
end

topicNamesUnique = string(cameraInfoStruct.topics)';

% first camera should be d_1
updatedOriginTopic = "/cam_d_1/infra1/image_rect_raw";
if ~strcmp(topicNamesUnique(1), updatedOriginTopic)
    for i = 1:length(topicNamesUnique)
        if strcmp(topicNamesUnique(i), updatedOriginTopic)
            actualOriginCamInd = i;
            break;
        end
    end

    actualOriginCamID = cameraIDsUnique(actualOriginCamInd);
    falseOriginCamID = cameraIDsUnique(1);

     actualOriginMask = cameraIDs==actualOriginCamID;
    faslseOriginMask = cameraIDs==falseOriginCamID;

    cameraIDs(actualOriginMask) = falseOriginCamID;
    cameraIDs(faslseOriginMask) = actualOriginCamID;

    topicNamesUnique(actualOriginCamInd) = topicNamesUnique(1);
    topicNamesUnique(1) = updatedOriginTopic;
end

% get short names from topics E.g. "/camera7/colour/image" becomes "camera7"
shortCamNameIndices = strfind(topicNamesUnique,"/");
shortCamName = topicNamesUnique;
shortCamNameDisplay = topicNamesUnique;

for i=1:length(topicNamesUnique)
    curNameChar = char(topicNamesUnique(i));
    curShortCamNameIndex = shortCamNameIndices{i};
    curNameChar = curNameChar(curShortCamNameIndex(1)+1:curShortCamNameIndex(2)-1);
    shortCamName(i) = curNameChar;
    curNameChar = strrep(curNameChar, '_', ' ');
    shortCamNameDisplay(i) = curNameChar;
end



camStart = min(cameraIDs);
% first camera ID should be 1
if camStart == 0
    cameraIDs = cameraIDs + 1;
    cameraIDsUnique = cameraIDsUnique + 1;
    camStart = 1;
elseif camStart > 1
    error("Invalid starting camera ID. No poses where found for the first camera (origin camera). Cannot carry out optimsation. Please re-collect calibration data.")
end
camEnd = max(cameraIDs);
numCam = length(cameraIDsUnique);

% display topics and IDS
tableUnique = table(cameraIDsUnique, topicNamesUnique, shortCamName, 'VariableNames',["Camera ID", "Camera Image Topic", "Optical frame name"]);
disp('');
disp(tableUnique);
fprintf("There are %i cameras. %s will be the Origin \n", numCam, shortCamName(1));

%% Determine board nodes using timestamps

disp("Determining board nodes using timestamps ...");

% convert poses to MATLAB format
for i = 1:size(poses,1)
   poses(i,:) = Convert2MatlabPoseOrder(poses(i, :)); 
end

[nodesMat,edgesMat] = getcameraboardposegraph(timeStamps, cameraIDs, poses);

%% Get initial extrinsic guesses from YAML

% extract translation [x,y,z] (in metres) and rotation [roll pitch yaw] (in degrees)
extrinsicGuessTrans = cell2mat(cameraInfoStruct.initial_extrinsic(:,1:3));
extrinsicGuessRot = deg2rad(cell2mat(cameraInfoStruct.initial_extrinsic(:,4:end)));
extrinsicGuessRot = eul2quat(extrinsicGuessRot, "ZYX");
nodesMat(1:numCam, 2:end) = [extrinsicGuessTrans, extrinsicGuessRot];

%% Setup pose graph

fprintf("Setting up pose graph ...")

numNodes = size(nodesMat, 1);
numEdges = size(edgesMat, 1);
poseGraph = poseGraph3D('MaxNumEdges',numEdges,'MaxNumNodes',numNodes);

%default information matrix (information matrix is inverse of covariance
%and accounts for sensor noise)
inforMatDefault = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];

%the reference node
fromNode = 1;

%add nodes to posegraph
for i = 2:numNodes
    curPose = nodesMat(i,2:end);
    addRelativePose(poseGraph, curPose, inforMatDefault, fromNode);
end

%add edges to posegraph
for i = 1:numEdges
    curPose = edgesMat(i,3:end);
    camNode = edgesMat(i,1);
    boardNode = edgesMat(i,2);
    addRelativePose(poseGraph, curPose, inforMatDefault, camNode, boardNode);
end

%plot graph before optimising
fname = 'graph_before_after_optimising';
hfig = figure('Name', 'Before Optimisation: nodes and edges');
tiledlayout(2,1, 'Padding', 'compact');

ax1 = nexttile;
show(poseGraph, 'IDs', 'off'); hold on;
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('Before Optimisation');
camIDoffset = 0;
camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(poseGraph, 1:camEnd)];

%plot cameras
for i = 1:numCam
    t = camNodeData(i,2:4);
    cam = camNodeData(i,1);
    text(t(1)+0.02, t(2)-0.02, t(3), num2str(cam), 'Color', 'blue', 'Clipping', 'on');
end

axis auto;

fprintf("DONE\n")
%% Perform optimisation

fprintf("Optimising pose graph ...")
optmPoseGraph = optimizePoseGraph(poseGraph, "g2o-levenberg-marquardt", "VerboseOutput",'on', "FunctionTolerance", 1e-15);
fprintf("DONE\n")

% plot graph after optimising
ax2 = nexttile;
show(optmPoseGraph, 'IDs', 'off');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
title('After Optimisation');

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(optmPoseGraph, 1:numCam)];

for i = 1:numCam
    t = camNodeData(i,2:4);
    cam = camNodeData(i,1);
    text(t(1)+0.02, t(2)-0.02, t(3), num2str(cam), 'Color', 'blue', 'Clipping', 'on');
end

linkaxes([ax1 ax2])
axis auto;

picturewidth = 20; % set this parameter and keep it forever
hw_ratio = 1.2; % feel free to play with this ratio


% set(findall(hfig,'-property','Box'),'Box','on') % optional
set(findall(hfig,'-property','FontSize'),'FontSize',14) % adjust fontsize to your document
set(findall(hfig,'-property','Interpreter'),'Interpreter','latex') 
set(findall(hfig,'-property','TickLabelInterpreter'),'TickLabelInterpreter','latex')
set(hfig,'Units','centimeters','Position',[3 3 picturewidth hw_ratio*picturewidth])
pos = get(hfig,'Position');
set(hfig,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])
% print(hfig,fullfile(csvDir, fname),'-dpng','-painters')

% close all;


% display results in table
varNames = {'Camera ID', 'tx', 'ty', 'tz', 'qw', 'qx', 'qy', 'qz'};

disp(' ');

resError = edgeResidualErrors(optmPoseGraph);
resErrorTotal = sum(resError, 'all');

disp('Initial guess of camera poses');
fprintf("Sum of residual errors: %d\n", resErrorTotal);

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(poseGraph, 1:numCam)];
disp(array2table(camNodeData,'VariableNames',  varNames));



hfig = figure('Name', 'UN-optimised camera poses');
R = eye(3);
t = [0,0,0];
pose = rigid3d(R,t);
plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', shortCamNameDisplay(1)); hold on;
trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
% set(findall(hfig,'-property','FontSize'),'FontSize',14) % adjust fontsize to your document


%plot cameras
for i = 2:numCam
    R = quat2rotm(camNodeData(i,5:end))';
    t = camNodeData(i,2:4);
    pose = rigid3d(R,t);
    plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', shortCamNameDisplay(i), 'Color', [0,0,1]); hold on;
    trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
end

grid on;
axis equal;
view(5,-45)










disp(' ');

disp('Optimised camera poses');
resErrorTotal = sum(edgeResidualErrors(optmPoseGraph), 'all');
fprintf("Sum of residual errors: %d\n", resErrorTotal);

camNodeData = [nodesMat(1:numCam,1), nodes(optmPoseGraph, 1:numCam)];
finalTable = array2table(camNodeData,'VariableNames',  varNames);
disp(finalTable);


boardNodeData = [nodesMat(numCam + 1:end,1),nodes(optmPoseGraph, numCam+1:numNodes)];



%plot boards
% for i=1:1:size(boardNodeData,1)
%     R = quat2rotm(boardNodeData(i,5:end));
%     t = boardNodeData(i,2:4);
%     pose = rigid3d(R,t);
%     trplot(pose.T', 'length', 0.05, 'thick', 2, 'rgb', 'notext', 'noarrow')
%     text(t(1), t(2), t(3), num2str(i));
% end

grid on;
axis equal;
view(5,-45)

picturewidth = 20; % set this parameter and keep it forever
hw_ratio = 0.6; % feel free to play with this ratio









fname = 'optimised_pose_graph.png';
hfig = figure('Name', 'Optimised camera poses');
R = eye(3);
t = [0,0,0];
pose = rigid3d(R,t);
plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', shortCamNameDisplay(1)); hold on;
trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
set(findall(hfig,'-property','FontSize'),'FontSize',14) % adjust fontsize to your document


%plot cameras
for i = 2:numCam
    R = quat2rotm(camNodeData(i,5:end))';
    t = camNodeData(i,2:4);
    pose = rigid3d(R,t);
    plotCamera('AbsolutePose', pose, 'Size', 0.05, 'AxesVisible', false, 'Label', shortCamNameDisplay(i), 'Color', [0,0,1]); hold on;
    trplot(pose.T', 'length', 0.1, 'thick', 2, 'rgb', 'notext', 'noarrow')
end

grid on;
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal;

%% save optimised parameters to YAML

camNodeData = [(camIDoffset+1:camIDoffset+numCam)', nodes(optmPoseGraph, 1:numCam)];
camROStransforms = zeros(numCam, 7);

for i = 1:numCam
    camPoseROS = Convert2ROSPoseOrder(camNodeData(i,2:end));
    camROStransforms(i,:) = camPoseROS;
end



dataText = saveTF2ROS_opticalframeFormat(camROStransforms, shortCamName);

yamlData.transforms = dataText;
yamlData.camera_topics = convertStringsToChars(topicNamesUnique);
yaml.WriteYaml(fullfile(calibrationDir, 'optimised_camera_poses.yaml'), yamlData);

function poseOut = Convert2MatlabPoseOrder(poseIn)
% Converts order of pose to match expected order in MATLAB.
% INPUT: pose [tx, ty, tz, qx, qy, qz, qw]
% OUTPUT: pose [tx, ty, tz, qw, qx, qy, qz]

poseOut = [poseIn(1:3), poseIn(7), poseIn(4:6)];
end

function poseOut = Convert2ROSPoseOrder(poseIn)
% Converts order of pose to match expected order in MATLAB.
% INPUT: pose  [tx, ty, tz, qw, qx, qy, qz]
% OUTPUT: pose [tx, ty, tz, qx, qy, qz, qw]

poseOut = [poseIn(1:3), poseIn(5:7), poseIn(4)];
end

function dataText = saveTF2ROS_opticalframeFormat(rosPoses, opticalFrameNames)

dataText = strings(length(opticalFrameNames)+1, 1);
    for i = 1:length(opticalFrameNames)
         curText = strcat(num2str(rosPoses(i,:),'%.6f '), " interim ", opticalFrameNames(i), "_depth_optical_frame");
         dataText(i) = curText;
         disp(curText);
    end

    dataText(end) = "0 0 0 -1.570796327 0 -1.570796327 world interim";

    dataText = convertStringsToChars(dataText);
end