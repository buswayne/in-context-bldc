function out = ndp_short(in) %#codegen

% A persistent object mynet is used to load the series network object.
% At the first call to this function, the persistent object is constructed and
% setup. When the function is called subsequent times, the same object is reused 
% to call predict on inputs, thus avoiding reconstructing and reloading the
% network object.

persistent mynet;

if isempty(mynet)
    mynet = coder.loadDeepLearningNetwork('ndp_short_H10H.mat');
end

% pass in input   
dl_in = single(dlarray(in, 'BTC'));
out = extractdata(predict(mynet,dl_in));

