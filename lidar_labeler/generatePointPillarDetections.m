function [boxPred,scores,labels] = generatePointPillarDetections(net,ptCloud,anchorBoxes,gridParams,boxClasses,confidenceThreshold,...
                                                 overlapThreshold,P,N,executionEnvironment)
% This function is used to obtain the predicted bounding boxes, class
% labels and confidence scores.

%   Copyright 2021 The MathWorks, Inc.

    ptCloud = cat(2,ptCloud.Location,ptCloud.Intensity);
    processedPtCloud = createPillars({ptCloud,'',''}, gridParams,P,N);

    % Extract the point cloud, pillar features and pillar indices.
    pillarFeatures = processedPtCloud{1,1};
    pillarIndices = processedPtCloud{1,2};

    % Convert to dlarray.
    XTestFeatures = dlarray(pillarFeatures, 'SSCB');
    XTestIndices = dlarray(pillarIndices, 'SSCB');

    % If GPU is available, then convert data to gpuArray.
    if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
        XTestFeatures = gpuArray(XTestFeatures);
        XTestIndices = gpuArray(XTestIndices);
    end                                         
                                             
    YPredictions = cell(size(net.OutputNames));
    
    % Predict the output of network and extract the following confidence,
    % x, y, z, l, w, h, yaw and class.
    [YPredictions{:}, ~] = predict(net,XTestIndices,XTestFeatures);

    boxPreds = generateDetections(YPredictions,gridParams,XTestIndices,...
                                                  anchorBoxes,confidenceThreshold);
                                              
    if ~isempty(boxPreds)                             
        posIdx = find(boxPreds(:,9) > 0.15);

        % Convert the yaw angle from radians to degrees.
        boxPreds(:,7) = rad2deg(boxPreds(:,7));

        if (executionEnvironment == "auto" && canUseGPU) || executionEnvironment == "gpu"
            boxPreds = gather(boxPreds);
        end

        if ~isempty(posIdx)  
            bboxRotRect = boxPreds(posIdx,[1,2,4,5,7]);
            scores = boxPreds(posIdx,8);
            labels = boxPreds(posIdx,9);

            [~,scores,labels,idx] = selectStrongestBboxMulticlass(bboxRotRect,scores,labels,...
                                         'RatioType','Min','OverlapThreshold',overlapThreshold);
            labels = categorical(boxClasses(labels));
            box3D = boxPreds(idx,:);

            % Convert the boxes to cuboid format.
            boxPred = zeros(size(box3D,1),9);
            boxPred(:,[1,2,3,4,5,6,9]) = box3D(:,1:7);

        else
            boxPred = [];
            scores = [];
            labels = [];
        end
    else
        boxPred = [];
        scores = [];
        labels = [];
    end
end

%% * Generating the point pillar detections * 
function boxPreds = generateDetections(YPredictions,gridParams,XTestIndices,...
                                                  anchorBoxes,confidenceThreshold)

    pillarIndices = gather(extractdata(XTestIndices));
    
    % Generate the tiled anchors from grid parameters and anchor boxes.
    [~, anchorsBEV] = createAnchors(gridParams, anchorBoxes);
    
    % Calculate the anchor mask from the pillar indices.
    anchorMask = createAnchorMask(pillarIndices, gridParams, anchorsBEV);
    
    % Find the anchors that has points inside them.
    anchorMask = anchorMask > 1;
    gridX = gridParams{1,4}{1}/gridParams{1,3}{3};
    gridY = gridParams{1,4}{2}/gridParams{1,3}{3};
    numAnchors = size(anchorBoxes,2);
    anchorMask = reshape(anchorMask,[numAnchors,gridX,gridY]);
    anchorMask = permute(anchorMask,[2,3,1]);
    
    YPredictions = cellfun(@gather, YPredictions, 'UniformOutput', false);
    YPredictions = cellfun(@extractdata, YPredictions, 'UniformOutput', false);
    
    % Extract the data from the dlarray.
    predAngle = YPredictions{1,6};
    predOcc = YPredictions{1,3};
    predLoc = reshape(YPredictions{1,2},[gridX,gridY,numAnchors,3]);
    predSz = reshape(YPredictions{1,1},[gridX,gridY,numAnchors,3]);
    predHeading = YPredictions{1,5};
    predClassification = YPredictions{1,4};
    
    posIndices = find((predOcc > confidenceThreshold) & (anchorMask));
    [row,col,anchorNum] = ind2sub(size(predOcc),posIndices);
    
    % Calculate the confidence score of the boxes.
    confScore = arrayfun(@(x,y,z) predOcc(x,y,z),row,col,anchorNum);
    
     % Calculate the box centre.
    xCen = (gridParams{1,3}{1})*(gridParams{1,3}{3})*(row-1) + gridParams{1,1}{1} + gridParams{1,3}{1};
    yCen = (gridParams{1,3}{2})*(gridParams{1,3}{3})*(col-1) + gridParams{1,1}{2} + gridParams{1,3}{2};
    
    % Decode the box locations.
    xGt = arrayfun(@(x,y,a,c) (predLoc(x,y,a,1)*(sqrt(anchorBoxes{1,a}{1}^2 + anchorBoxes{1,a}{2}^2))+c),row,col,anchorNum,xCen);
    yGt = arrayfun(@(x,y,a,c) (predLoc(x,y,a,2)*(sqrt(anchorBoxes{1,a}{1}^2 + anchorBoxes{1,a}{2}^2))+c),row,col,anchorNum,yCen);
    zGt = arrayfun(@(x,y,a,c) (predLoc(x,y,a,3)*anchorBoxes{1,a}{3} + anchorBoxes{1,a}{4}), row,col,anchorNum);
    zGt = 0.5*zGt;
    
    % Decode the box dimensions.
    lGt = arrayfun(@(x,y,a) (exp(predSz(x,y,a,1))*anchorBoxes{1,a}{1}),row,col,anchorNum);
    wGt = arrayfun(@(x,y,a) (exp(predSz(x,y,a,2))*anchorBoxes{1,a}{2}),row,col,anchorNum);
    hGt = arrayfun(@(x,y,a) (exp(predSz(x,y,a,3))*anchorBoxes{1,a}{3}),row,col,anchorNum);
    
    hdGt = arrayfun(@(x,y,a) predHeading(x,y,a),row,col,anchorNum);
    hdGt(hdGt >= 0.5) = 1;
    hdGt(hdGt < 0.5) = -1;
    
    % Decode the box yaw angle.
    predAngle((predAngle > 1) | (predAngle < -1)) = 0;
    angGt = arrayfun(@(x,y,a,h) (h*asin(predAngle(x,y,a))+anchorBoxes{1,a}{5}),row,col,anchorNum,hdGt);
    angGt = iWrapToPi(angGt);
   
    predClassification = reshape(predClassification, gridX, gridY, numAnchors, []); 
    numClasses = size(predClassification, 4);
    
    % Find the class of the object.
    if(numClasses > 1) 
        cls = arrayfun(@(x,y,a) squeeze(predClassification(x,y,a,:))',row,col,anchorNum, 'UniformOutput', false);
        clsMat = cell2mat(cls);
        [~, cls] = max(clsMat,[],2);        
    else
        cls = arrayfun(@(x,y,a) predClassification(x,y,a),row,col,anchorNum);
        cls = 1./(1.+exp(-cls));
        cls(cls >= confidenceThreshold) = 1;
        cls(cls < confidenceThreshold) = 0;
    end
    
    boxPreds = [xGt,yGt,zGt,lGt,wGt,hGt,angGt,confScore,cls];
end

%% * Wrap from [-pi,pi] *
function alpha = iWrapToPi(alpha)
    idx = alpha > pi;
    alpha(idx) = alpha(idx)-2*pi;

    idx = alpha < -pi;
    alpha(idx) = alpha(idx)+2*pi;
end
