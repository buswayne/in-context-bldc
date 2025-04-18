function [error] = cost_function_CL(var)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:200,:);
mask = (real_data(:,1) >= 0.2 & real_data(:,1) <= 0.4) | (real_data(:,1) >= 1.2 & real_data(:,1) <= 1.5);
i_r = var.i_r;
i_l = var.i_l;
i_f = var.i_f;
i_i = var.i_i;
i_bv = var.i_b;

mdl = 'BLDC_simulator_BO';

simIn = Simulink.SimulationInput(mdl);
simIn = setVariable(simIn,'i_r', i_r);
simIn = setVariable(simIn,'i_l', i_l);
simIn = setVariable(simIn,'i_f', i_f);
simIn = setVariable(simIn,'i_i', i_i);
simIn = setVariable(simIn,'i_bv', i_bv);
simout = sim(simIn);

% sim(mdl)

error = immse(simout.output.signals.values(mask,2), real_data(mask,6));

% error = error^2;
end