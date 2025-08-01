clear
close all

path = '../speed_controller/ndp_noise_h10_40k_weights.pkl';

fid = py.open(path,'rb');
data = py.pickle.load(fid);
fid.close()

%% model config


n_layer = double(data{'n_layer'});
block_size = double(data{'block_size'});
n_embd = double(data{'n_embd'});
n_u = double(data{'n_u'});
n_y = double(data{'n_y'});
n_head = double(data{'n_head'});

%% test input output


test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());

%% net structure

net = dlnetwork;
layer_list = [];

input_layer = inputLayer([1, block_size, n_u], "BTC", 'Name','input_layer');
layer_list = [layer_list input_layer];


wte = fullyConnectedLayer(n_embd, "Name",'wte');
wte_w = double(data{'transformer.wte.weight'}.cpu().numpy());
wte_b = double(data{'transformer.wte.bias'}.cpu().numpy());
wte.Weights = wte_w;
wte.Bias = wte_b';
layer_list = [layer_list wte];

wpe = positionEmbeddingLayer(n_embd, block_size, "Name",'wpe');
wpe_w = double(data{'transformer.wpe.weight'}.cpu().numpy());
wpe.Weights = wpe_w';
layer_list = [layer_list wpe];


last_name = 'add_init';
add = additionLayer(2, "Name",last_name);
layer_list = [layer_list add];


net = addLayers(net, layer_list);
net = connectLayers(net, 'wte', 'add_init/in2');

plot(net)

% layer_list = [];

for h = 0:n_layer-1

    h_name = string(h);

    ln1 = layerNormalizationLayer("Name",'ln1_block_'+h_name);
    ln1_scale = double(data{'transformer.h.' + h_name + '.ln_1.weight'}.cpu().numpy());
    ln1.Scale = ln1_scale;
    
    att_layer = selfAttentionLayer(n_head,n_embd,'AttentionMask','causal', 'Name','attn_block_'+h_name);
    c_attn_w = double(data{'transformer.h.' + h_name + '.attn.c_attn.weight'}.cpu().numpy());
    c_proj_w = double(data{'transformer.h.' + h_name + '.attn.c_proj.weight'}.cpu().numpy());
    [len, wid] = size(c_attn_w);
    c_attn_w_q = c_attn_w(1:wid,:);
    c_attn_w_k = c_attn_w((wid+1):2*wid,:);
    c_attn_w_v = c_attn_w((2*wid+1):3*wid,:);
    att_layer.OutputWeights = c_proj_w;
    att_layer.QueryWeights = c_attn_w_q;
    att_layer.KeyWeights = c_attn_w_k;
    att_layer.ValueWeights = c_attn_w_v;

    %diomerda
    add_1 = additionLayer(2, "Name", "add_1_block_"+h_name);

    ln2 = layerNormalizationLayer("Name",'ln2_block_'+h_name);
    ln2_scale = double(data{'transformer.h.' + h_name + '.ln_2.weight'}.cpu().numpy());
    ln2.Scale = ln2_scale;
    
    % MLP
    layer_pre = fullyConnectedLayer(4*n_embd, 'Name','MLP_pre_block_'+h_name);
    c_fc = double(data{'transformer.h.' + h_name + '.mlp.c_fc.weight'}.cpu().numpy());
    layer_pre.Weights = c_fc;

    gelu_layer = geluLayer("Approximation","none", 'Name','gelu_block_'+h_name);

    layer_post = fullyConnectedLayer(n_embd, 'Name','MLP_post_block_'+h_name);
    c_proj = double(data{'transformer.h.' + h_name + '.mlp.c_proj.weight'}.cpu().numpy());
    layer_post.Weights = c_proj;

    add_2 = additionLayer(2, "Name", "add_2_block_"+h_name);

    % net = addLayers(net, [ln1, att_layer, ln2, layer_pre, gelu_layer, layer_post]);
    layer_list = [ln1, att_layer, add_1, ln2, layer_pre, gelu_layer, layer_post, add_2]; 
    net = addLayers(net, layer_list);
    net = connectLayers(net, last_name, 'ln1_block_'+h_name);
    net = connectLayers(net, last_name, "add_1_block_"+h_name + '/in2');
    net = connectLayers(net, "add_1_block_"+h_name, "add_2_block_"+h_name + '/in2');
    last_name = "add_2_block_"+h_name;
end



lnf = layerNormalizationLayer("Name", 'lnf');
lnf_scale = double(data{'transformer.ln_f.weight'}.cpu().numpy());
lnf.Scale = lnf_scale;

layer_final = fullyConnectedLayer(n_y, 'Name', 'layer_final');
layer_final_w = double(data{'lm_head.weight'}.cpu().numpy());
layer_final_b = double(data{'lm_head.bias'}.cpu().numpy());
layer_final.Weights = layer_final_w;
layer_final.Bias = layer_final_b;

layer_list = [lnf, layer_final];
net = addLayers(net, layer_list);
net = connectLayers(net, last_name, 'lnf');

% 
% 
% 
% net = addLayers(net, layer_list);
% net = connectLayers(net, 'wte', 'add/in2');
% net = addLayers(net, layer_list);
% net = connectLayers(net, 'add', 'ln1_block_0');

plot(net)

% 
net = initialize(net);
output = extractdata(predict(net,test_input));


error = abs(test_output-output);
mean(reshape(error,1,[]))


in_2 = double(data{'test_input'}.detach().numpy());

[a,b,c]= size(in_2);
out_2 = output*0;
for jj = 1:a
    out_2(jj,:) = net_predict_mex(in_2(jj,:,:));
end



error = abs(test_output-out_2);
mean(reshape(error,1,[]))