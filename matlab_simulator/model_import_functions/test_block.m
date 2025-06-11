%%% pickle -> matrix
% fid=py.open('test_pkl.pkl','rb');
% data=py.pickle.load(fid);
% a = data{'transformer.h.0.ln_2.weight'}
% mat = double(a.cpu().numpy())
clear
close all

fid = py.open('../../speed_controller/test_B.pkl','rb');
data = py.pickle.load(fid);
fid.close()

% data.keys()

block_size = double(data{'block_size'});
n_embd = double(data{'n_embd'});
n_head = double(data{'n_head'});


input_layer = inputLayer([1 block_size n_embd], "BTC",'Name','input_layer');

ln1 = layerNormalizationLayer("Name",'ln1_block_');
ln1_scale = double(data{'ln_1.weight'}.cpu().numpy());
ln1.Scale = ln1_scale;

att_layer = selfAttentionLayer(n_head,n_embd,'AttentionMask','causal', 'Name','attn_block_');
c_attn_w = double(data{'attn.c_attn.weight'}.cpu().numpy());
c_proj_w = double(data{'attn.c_proj.weight'}.cpu().numpy());
[len, wid] = size(c_attn_w);
c_attn_w_q = c_attn_w(1:wid,:);
c_attn_w_k = c_attn_w((wid+1):2*wid,:);
c_attn_w_v = c_attn_w((2*wid+1):3*wid,:);
att_layer.OutputWeights = c_proj_w;
att_layer.QueryWeights = c_attn_w_q;
att_layer.KeyWeights = c_attn_w_k;
att_layer.ValueWeights = c_attn_w_v;

ln2 = layerNormalizationLayer("Name",'ln2_block_');
ln2_scale = double(data{'ln_2.weight'}.cpu().numpy());
ln2.Scale = ln2_scale;

add_1 = additionLayer(2,"Name",'add_1');

% MLP
layer_pre = fullyConnectedLayer(4*n_embd, 'Name','MLP_pre_block_');
c_fc = double(data{'mlp.c_fc.weight'}.cpu().numpy());
layer_pre.Weights = c_fc;

gelu_layer = geluLayer("Approximation","none", 'Name','gelu_block_');


layer_post = fullyConnectedLayer(n_embd, 'Name','MLP_post_block_');
c_proj = double(data{'mlp.c_proj.weight'}.cpu().numpy());
layer_post.Weights = c_proj; 

add_2 = additionLayer(2,"Name",'add_2');

test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());



layer_list = [input_layer, ln1, att_layer, add_1, ln2, layer_pre, gelu_layer, layer_post, add_2];


net = dlnetwork();
net = addLayers(net, layer_list);
net = connectLayers(net,'input_layer', 'add_1/in2');
net = connectLayers(net,'add_1', 'add_2/in2');

% plot(net)
net = initialize(net);
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
