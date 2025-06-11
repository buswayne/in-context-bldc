%%% pickle -> matrix
% fid=py.open('test_pkl.pkl','rb');
% data=py.pickle.load(fid);
% a = data{'transformer.h.0.ln_2.weight'}
% mat = double(a.cpu().numpy())
clear
close all

fid = py.open('../../speed_controller/test_MLP.pkl','rb');
data = py.pickle.load(fid);
fid.close()

% data.keys()

c_fc = double(data{'c_fc.weight'}.cpu().numpy());
c_proj = double(data{'c_proj.weight'}.cpu().numpy());
block_size = double(data{'block_size'});
n_embd = double(data{'n_embd'});


test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());


input_layer = inputLayer([1 block_size n_embd], "BTC");
layer_pre = fullyConnectedLayer(4*n_embd);
layer_pre.Weights = c_fc;
gelu = geluLayer("Approximation","none");
layer_post = fullyConnectedLayer(n_embd);
layer_post.Weights = c_proj;


net = dlnetwork([input_layer, layer_pre, gelu, layer_post]);

% input_tmp = ones(1,10,16);
% input = dlarray(input_tmp);

output = extractdata(predict(net,test_input));

% for i = 1:16
%     output(:,:,i)
% end
% 
% input_tmp = zeros(1,10,16);
% input = dlarray(input_tmp);
% 
% output2 = extractdata(predict(net,input));



error = test_output-output;
mean(reshape(error,1,[]))