clear
close all

path = '../../speed_controller/models_to_mat/old/test_chopped_weights.pkl';

fid = py.open(path,'rb');
data = py.pickle.load(fid);
fid.close();
keys = data.keys();

%% model config


n_layer = double(data{'n_layer'});
block_size = double(data{'block_size'});
n_embd = double(data{'n_embd'});
n_u = double(data{'n_u'});
n_y = double(data{'n_y'});

%% test input output


test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());

%% net weight


wte_w = double(data{'transformer.wte.weight'}.cpu().numpy());
wte_b = double(data{'transformer.wte.bias'}.cpu().numpy());
wpe_w = double(data{'transformer.wpe.weight'}.cpu().numpy());

%% net structure

net = dlnetwork;

input_layer = inputLayer([1, block_size, n_u], "BTC", "Name",'input_layer');

% wpe = positionEmbeddingLayer(n_embd, block_size);
% wpe.Weights = wpe_w';

wte = fullyConnectedLayer(n_embd, "Name",'wte');
wte.Weights = wte_w;
wte.Bias = wte_b';
wpe = positionEmbeddingLayer(n_embd, block_size, "Name",'wpe');
wpe.Weights = wpe_w';
add = additionLayer(2, "Name",'add');

net = addLayers(net, [input_layer, wte, wpe, add]);
% net = addLayers(net, [input_layer, wpe]);
% net = connectLayers(net, 'input_layer', 'wte');
% net = connectLayers(net, 'wte', 'wpe');
net = connectLayers(net, 'wte', 'add/in2');
% net = connectLayers(net, 'wpe', 'add/in2');

plot(net)
net = initialize(net);


output = extractdata(predict(net,test_input));






