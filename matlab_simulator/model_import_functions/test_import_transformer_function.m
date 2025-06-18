clear
close all
clc

path = '../../speed_controller/new_delay_h10_10k_weights.pkl';
net = import_transformer_model(path);