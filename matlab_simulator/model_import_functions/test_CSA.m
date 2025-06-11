%%% pickle -> matrix
% fid=py.open('test_pkl.pkl','rb');
% data=py.pickle.load(fid);
% a = data{'transformer.h.0.ln_2.weight'}
% mat = double(a.cpu().numpy())
clear
close all

fid = py.open('../../speed_controller/test_CSA.pkl','rb');
data = py.pickle.load(fid);
fid.close()

% data.keys()


test_input = dlarray(double(data{'test_input'}.detach().numpy()));
test_output = double(data{'test_output'}.detach().numpy());

c_attn_w = double(data{'c_attn.weight'}.cpu().numpy());
c_proj_w = double(data{'c_proj.weight'}.cpu().numpy());

[len, wid] = size(c_attn_w);
c_attn_w_q = c_attn_w(1:wid,:);
c_attn_w_k = c_attn_w((wid+1):2*wid,:);
c_attn_w_v = c_attn_w((2*wid+1):3*wid,:);

input_layer = inputLayer([1 10 16], "BTC");
att_layer = selfAttentionLayer(4,16,'AttentionMask','causal');

att_layer.OutputWeights = c_proj_w;
att_layer.QueryWeights = c_attn_w_q;
att_layer.KeyWeights = c_attn_w_k;
att_layer.ValueWeights = c_attn_w_v;

net = dlnetwork([input_layer, att_layer]);

input_tmp = ones(1,10,16);
input = dlarray(input_tmp);

output = extractdata(predict(net,input));
% 
% for i = 1:16
%     output(:,:,i)
% end


error = test_output-output;
mean(reshape(error,1,[]))

