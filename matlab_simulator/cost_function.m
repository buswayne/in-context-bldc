function [error] = cost_function(var)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
temp_name = strsplit(pwd,'in-context-bldc');
savepath = fullfile(temp_name{1}, "in-context-bldc","data","simulated\CL_speed_matlab\");
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:200,:);
i_r = var(1);
i_l = var(2);
i_f = var(3);
i_i= var(4);
i_bv = var(5);

mdl = 'BLDC_simulator_BO';

simIn = Simulink.SimulationInput(mdl);
simIn = setVariable(simIn,'i_r', i_r);
simIn = setVariable(simIn,'i_l', i_l);
simIn = setVariable(simIn,'i_f', i_f);
simIn = setVariable(simIn,'i_i', i_i);
simIn = setVariable(simIn,'i_bv', i_bv);
simout = sim(simIn);

% sim(mdl)

error = immse(simout.output.signals.values(:,5), real_data(:,2)) + immse(simout.output.signals.values(:,4), real_data(:,3));

end