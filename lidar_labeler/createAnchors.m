function [anchors3D,anchorsBEV] = createAnchors(gridParams, anchorBoxes)
% This function creates set of anchors for the entire point cloud given the point cloud
% range and anchor dimensions for the objects to be detected.

%   Copyright 2021 The MathWorks, Inc.

    voxelSize = [gridParams{1,3}{1}, gridParams{1,3}{2}];
    coorsRange = [gridParams{1,1}{1}, gridParams{1,1}{2}, gridParams{1,1}{3}..., 
                  gridParams{1,2}{1}, gridParams{1,2}{2}, gridParams{1,2}{3}];
    downsamplingFactor = gridParams{1,3}{3};
    anchors = [];
    for i = 1:numel(anchorBoxes)
        tmp = [anchorBoxes{1,i}{1}, anchorBoxes{1,i}{2}, anchorBoxes{1,i}{3}, anchorBoxes{1,i}{4}, anchorBoxes{1,i}{5}];
        anchors = [anchors;tmp];
    end
    numAnchors = size(anchors,1);

    xStep = voxelSize(1,1) * downsamplingFactor;
    yStep = voxelSize(1,2) * downsamplingFactor;
    gridX = (coorsRange(1,4) - coorsRange(1,1))/xStep;
    gridY = (coorsRange(1,5) - coorsRange(1,2))/yStep;

    x = linspace(coorsRange(1,1)+xStep*0.5 , coorsRange(1,4)-xStep*0.5, gridX);
    y = linspace(coorsRange(1,2)+yStep*0.5 , coorsRange(1,5)-yStep*0.5, gridY);
    [yy,xx] = meshgrid(y,x);
    xxT = reshape(xx,[numel(xx),1]);
    yyT = reshape(yy,[numel(yy),1]);
    tmp = cat(2,xxT,yyT);
    tmp = repelem(tmp,numAnchors,1);

    anchors3D = zeros([gridX*gridY*numAnchors,7]);
    anchors3D(:,1:2) = tmp;
    anchorsDim = repmat(anchors,[gridX*gridY,1]);
    anchors3D(:,3) = anchorsDim(:,4);
    anchors3D(:,4:6) = anchorsDim(:,1:3);
    anchors3D(:,7) = anchorsDim(:,5);

    anchors3dMod = anchors3D;
    anchors3dMod(2:2:size(anchors3D,1),[4,5]) = anchors3dMod(2:2:size(anchors3D,1),[5,4]);
    tmp1 = anchors3dMod(:,[1,2]) - anchors3dMod(:,[4,5])/2.0;
    tmp2 = anchors3dMod(:,[1,2]) + anchors3dMod(:,[4,5])/2.0;
    anchorsBEV = cat(2,tmp1,tmp2);
    
end
