clear
close all
clc

path = '../../speed_controller/normal_and_disturbed_h10_40k_weights.pkl';
net = import_transformer_model(path);