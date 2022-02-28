%% Generate anchor mask 

%   Copyright 2021 The MathWorks, Inc.
function anchorMask = createAnchorMask(pillarIndices, gridParams, anchorsBEV)
    % This function creates anchor mask for the anchors given the pillar Indices, point
    % cloud range and the bird's eye view dimensions of the anchors.
    sparseVoxelMap = zeros(gridParams{1,4}{1},gridParams{1,4}{2});
    numPillars = nnz(pillarIndices)/2;
    pillarIndices = pillarIndices(1:numPillars,:);
    sparseVoxelMap(sub2ind([gridParams{1,4}{1},gridParams{1,4}{2}],pillarIndices(:,1),pillarIndices(:,2))) = 1;

    sparseVoxelMap = cumsum(sparseVoxelMap, 1);
    sparseVoxelMap = cumsum(sparseVoxelMap, 2);
    anchorMask = zeros(size(anchorsBEV,1),1);
    
    for k = 1:size(anchorsBEV,1)

        % Calculating Bird's eye view dimensions of anchor
        bevXmin = anchorsBEV(k,1);
        bevYmin = anchorsBEV(k,2);
        bevXmax = anchorsBEV(k,3);
        bevYmax = anchorsBEV(k,4);

        xminGrid = floor((bevXmin - gridParams{1,1}{1})/gridParams{1,3}{1});
        yminGrid = floor((bevYmin - gridParams{1,1}{2})/gridParams{1,3}{2});
        xmaxGrid = floor((bevXmax - gridParams{1,1}{1})/gridParams{1,3}{1});
        ymaxGrid = floor((bevYmax - gridParams{1,1}{2})/gridParams{1,3}{2});

        xminGrid = max(xminGrid, 1);
        yminGrid = max(yminGrid, 1);
        xmaxGrid = min(xmaxGrid, gridParams{1,4}{1});
        ymaxGrid = min(ymaxGrid, gridParams{1,4}{2});

        ID = sparseVoxelMap(xmaxGrid, ymaxGrid);
        IA = sparseVoxelMap(xminGrid, yminGrid);
        IB = sparseVoxelMap(xminGrid, ymaxGrid);
        IC = sparseVoxelMap(xmaxGrid, yminGrid);

        anchorMask(k,1) = ID - IC - IB + IA;
    end
end
