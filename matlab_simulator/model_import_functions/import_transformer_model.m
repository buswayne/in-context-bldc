function net = import_transformer_model(path)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
fid = py.open(path,'rb');
data = py.pickle.load(fid);
fid.close()


end