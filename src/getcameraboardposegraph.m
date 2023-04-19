function [nodes,edges] = getcameraboardposegraph(timeStamps, cameraIDs, poses, timeDiffThreshold, timeBetweenBoard, minNumBoardsGroup, camOriginID)
% Get board nodes using timeStamps from estimated camera extrinsics. It
% works by assuming that estimated board poses within a window of time
% are captured when the board is in the exact same pose. For each
% board node, there will exist only one unique pose w.r.t. each camera. A
% board node is only useful if it has atleast 2 camera node edges. The
% edges with each board node can vary.
% A board pose is an estimated extrinsic w.r.t. a single camera. A board
% node is the board pose w.r.t. the origin camera at a single time-step.
% INPUTS:
%       timeStamps - timestamps of estimated poses
%       cameraIDs - w.r.t. camera ID of estimated poses
%       poses - poses as [x,y,z,qw,qx,qy,qz]
%       timeDiffThreshold - Maximum time (in seconds) where measured board
%       poses are considered to be the same board node.
%       timeBetweenBoard - Maximum time (in seconds) between boards. This 
%       ensures board nodes are not to close together to avoid overlapping.
%       camOriginID - Camera ID thats considered the origin
% OUTPUTS:
%       nodes - board node ID and its pose (initalised to be at the origin). 
%       First board node ID will always be the last camera node ID + 1
%       edges - poses of a board w.r.t. a single camera. 
%
% Author: Jasprabhjit Mehami, 16/04/2023

arguments
    timeStamps;
    cameraIDs;
    poses;
    timeDiffThreshold = 0.1;
    timeBetweenBoard = 0.2;
    minNumBoardsGroup = 2;
    camOriginID = 1;
end

%sort timestamps
[timeStamps, sortedIdxs] = sort(timeStamps);
cameraIDs = double(cameraIDs(sortedIdxs));
maxCameraIDs = max(cameraIDs);

numPoses = length(timeStamps);
boardIdxGroupedCell = cell(1, numPoses); % Board IDs assigned to each group
camIdGroupedCell = cell(1, numPoses); %cam IDs assigned to each group
numGroups = 1;
numPosesGroup = zeros(1,numPoses); % Number of poses for each group

%initialising variables with first pose
boardIdxGroupedCell(1) = {sortedIdxs(1)};
camIdGroupedCell(1) = {cameraIDs(1)};
groupStartTimeStamp = timeStamps(1);
numPosesGroup(1) = 1;

% Go through each pose and check if within timeDiffThreshold to assign it 
% to the current group, else start a new group.

for i = 2:numPoses
    curTimeStamp = timeStamps(i);
    curIdx = sortedIdxs(i);
    curCamID = cameraIDs(i);

    if abs(curTimeStamp - groupStartTimeStamp) < timeDiffThreshold
        groupIdxs = boardIdxGroupedCell{numGroups};
        groupIdxs = [groupIdxs, curIdx];
        boardIdxGroupedCell(numGroups) = {groupIdxs};

        camIds = camIdGroupedCell{numGroups};
        camIds = [camIds, curCamID];
        camIdGroupedCell(numGroups) = {camIds};
        numPosesGroup(numGroups) = numPosesGroup(numGroups) + 1;
    elseif abs(groupStartTimeStamp - curTimeStamp) > timeBetweenBoard
        numGroups = numGroups + 1;
        boardIdxGroupedCell(numGroups) = {curIdx};
        camIdGroupedCell(numGroups) =  {curCamID};
        groupStartTimeStamp = curTimeStamp;
        numPosesGroup(numGroups) = 1;
    end
end

boardIdxGroupedCell = boardIdxGroupedCell(1:numGroups);
camIdGroupedCell =  camIdGroupedCell(1:numGroups);
numPosesGroup =  numPosesGroup(1:numGroups);

for i = 1:numGroups
    groupIdxs = boardIdxGroupedCell{i};
    camIds = camIdGroupedCell{i};

    [C,ia] = unique(camIds);

    camIdGroupedCell(i) = {C};
    boardIdxGroupedCell(i) = {groupIdxs(ia)};
    numPosesGroup(i) = length(ia);
end

maskGroup = numPosesGroup >= minNumBoardsGroup;
numPosesGroup = numPosesGroup(maskGroup);
numPoses = sum(numPosesGroup);
numGroups = sum(maskGroup);
boardIdxGroupedCell = boardIdxGroupedCell(maskGroup);
camIdGroupedCell = camIdGroupedCell(maskGroup);

edges = zeros(numPoses, 9);
idx = 1;

%populate edges
for i = 1:numGroups
    groupIdxs = boardIdxGroupedCell{i};
    camIds = camIdGroupedCell{i};
    boardID = i + maxCameraIDs;
    curNumGroupPoses = numPosesGroup(i);
    edges(idx:idx+curNumGroupPoses-1, :) = [camIds', boardID.*ones(size(camIds')), poses(groupIdxs, :) ];
    idx = idx + curNumGroupPoses;
end

%populate nodes
nodes = zeros(numGroups + maxCameraIDs - 1, 8);
nodes(:,5) = 1;
nodes(1,1) = 1;

for i = 2:maxCameraIDs
    %     randTF = -1 + 2*rand(1,7);
    %     randTF = [zeros(1,6), 1];
    nodes(i,1) = i;
end

originBoardMask = edges(:,1)==camOriginID;
posesOrigin = edges(originBoardMask, :);

for i = maxCameraIDs+1:maxCameraIDs+numGroups
    diffID = abs(posesOrigin(:,2)-i);
    [~, nearestBoardIdx] = min(diffID);
    nearestBoardIdx = nearestBoardIdx(1);
    nodes(i,:) = [i,posesOrigin(nearestBoardIdx, 3:end)];
end
end