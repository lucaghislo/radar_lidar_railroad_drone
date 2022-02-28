function classificationInfo = helperTrueClassificationInfo(detections)
% This is a helper function and may be removed or modified in a future
% release.

% This function returns the true classification information about the
% detections for the multipath radar tracking example. It assumes there are
% 5 vehicles in the scene including ego.

if iscell(detections)
    tgtIdx = cellfun(@(x)x.ObjectAttributes{1}.TargetIndex, detections);
    pathIdx = cellfun(@(x)x.ObjectAttributes{1}.BouncePathIndex, detections);
    bounceIdx = cellfun(@(x)x.ObjectAttributes{1}.BounceTargetIndex, detections);
else
    tgtIdx = arrayfun(@(x)x.ObjectAttributes.TargetIndex, detections);
    pathIdx = arrayfun(@(x)x.ObjectAttributes.BouncePathIndex, detections);
    bounceIdx = arrayfun(@(x)x.ObjectAttributes.BounceTargetIndex, detections);
end

isReal = tgtIdx > 0 & tgtIdx <= 5;
isClutter = tgtIdx < 0;

isTarget = isReal & pathIdx == 0;
isStaticGhost = isReal & pathIdx > 0 & bounceIdx > 5;
isDynamicGhost = isReal & pathIdx > 0 & bounceIdx <= 5;
isStatic = tgtIdx > 5;

classificationInfo = zeros(numel(detections),1);

classificationInfo(isTarget) = 1;
classificationInfo(isStaticGhost) = 2;
classificationInfo(isDynamicGhost) = 3;
classificationInfo(isStatic) = 4;
classificationInfo(isClutter) = 5;

end
