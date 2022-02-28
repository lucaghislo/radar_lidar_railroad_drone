function mat = helperConfusionMatrix(true, predicted)
% This is a helper function and may be removed or modified in a future
% release. 

% Copyright 2020 The MathWorks, Inc.

% This function calculates the confusion matrix for radar classification in
% the multipath radar tracking example. 

mat = zeros(5,5);

for i = 1:5
    for j = 1:5
        mat(i,j) = sum(true == i & predicted == j);
    end
end

end
