clear
close all
clc


temp_name = strsplit(pwd,'in-context-bldc');
temp_name2 = fullfile(temp_name{1},'in-context-bldc/speed_controller/models_to_mat');

model_name = 'ndp_random_correction_long_weights.pkl';
model_name_to_save = 'ndp_random_correction_long';

network_path = fullfile(temp_name2, model_name);
save_path = fullfile(temp_name{1},'in-context-bldc/matlab_simulator/networks');
[net, H] = import_transformer_model(network_path);
model_name_to_save = model_name_to_save + "_H" + num2str(H) + "H" + ".mat";
save(fullfile(save_path, model_name_to_save), "net")