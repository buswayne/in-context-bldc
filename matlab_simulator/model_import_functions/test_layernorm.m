%%% pickle -> matrix
% fid=py.open('test_pkl.pkl','rb');
% data=py.pickle.load(fid);
% a = data{'transformer.h.0.ln_2.weight'}
% mat = double(a.cpu().numpy())
clear
close all

fid = py.open('../../speed_controller/test_LN.pkl','rb');
data = py.pickle.load(fid);
fid.close()

% data.keys()

w = double(data{'weight'}.cpu().numpy());
block_size = double(data{'block_size'});
n_embd = double(data{'n_embd'});

lnf = layerNormalizationLayer("Name", 'lnf');
lnf.Scale = w;


test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());


input_layer = inputLayer([1 block_size n_embd], "BTC");


net = dlnetwork([input_layer, lnf]);
% net = dlupdate(@double, net);
% net.State = dlupdate(@double, net.State);

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
