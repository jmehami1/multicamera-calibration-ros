function [nodes,edges] = getcameraboardposegraph(timeStamps, cameraIDs, poses)
% Get board nodes using timeStamps from estimated camera extrinsics. It
% works by assuming that estimated board poses within a window of time
% are captured when the board is in the exact same pose. For each
% board node, there will exist only one unique pose w.r.t. each camera.
% INPUTS:
%       timeStamps - timestamps of estimated poses
%       cameraIDs - w.r.t. camera ID of estimated poses
%       poses - poses as [x,y,z,qw,qx,qy,qz]
% OUTPUTS:
%       nodes - board node ID and its pose (initalised to be at the origin). First node ID will always be the last camera
%               ID + 1
%       edges - 
%           


%Maximum time difference threshold where a measured relative pose of
%the board w.r.t to a camera is considered to be the same pose of the
%board
timeDiffThreshold = 0.1;
timeBetweenBoard = 0.2;
minNumBoardsGroup = 2;
camOriginID = 1;
%sort timestamps

[timeStamps, sortedIdxs] = sort(timeStamps);
cameraIDs = double(cameraIDs(sortedIdxs));
maxCameraIDs = max(cameraIDs);

numPoses = length(timeStamps);
boardIdxGroupedCell = cell(1, numPoses);
camIdGroupedCell = cell(1, numPoses);
numGroups = 1;
numPosesGroup = zeros(1,numPoses);

boardIdxGroupedCell(1) = {sortedIdxs(1)};
camIdGroupedCell(1) = {cameraIDs(1)};
groupStartTimeStamp = timeStamps(1);
numPosesGroup(1) = 1;

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