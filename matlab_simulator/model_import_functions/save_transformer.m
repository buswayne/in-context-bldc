clear
close all
clc


temp_name = strsplit(pwd,'in-context-bldc');
temp_name2 = fullfile(temp_name{1},'in-context-bldc/speed_controller_v2/pkl_models');

model_name = 'new_dataset_long_noise_h10_40k_weights.pkl';
model_name_to_save = 'new_dataset_long_noise_h10_40k';

network_path = fullfile(temp_name2, model_name);
save_path = fullfile(temp_name{1},'in-context-bldc/matlab_simulator/networks2');
[net, H] = import_transformer_model(network_path);
model_name_to_save = model_name_to_save + "_H" + num2str(H) + "H" + ".mat";
save(fullfile(save_path, model_name_to_save), "net")