function [error] = cost_function3_alt(var)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

temp_name = strsplit(pwd,'in-context-bldc');

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:1000,:);
mask = abs(real_data(:,2)) >= 1;
% mask = (real_data(:,1) >= 0.2 & real_data(:,1) <= 0.4) | (real_data(:,1) >= 1.2 & real_data(:,1) <= 1.5);

i_r = var.i_r;
i_l = var.i_l;
i_f = var.i_f;
i_i = var.i_i;
i_omega = var.i_omega;

mdl = 'BLDC_simulator_BO_alt';

simIn = Simulink.SimulationInput(mdl);
simIn = setVariable(simIn,'i_r', i_r);
simIn = setVariable(simIn,'i_l', i_l);
simIn = setVariable(simIn,'i_f', i_f);
simIn = setVariable(simIn,'i_i', i_i);
simIn = setVariable(simIn,'i_omega', i_omega);
simout = sim(simIn);

% sim(mdl)
% e1 = immse(simout.output.signals.values(mask,5), real_data(mask,2))
% e2 = immse(simout.output.signals.values(mask,4), real_data(mask,3))
% e3 = immse(simout.output.signals.values(mask,2), real_data(mask,6))
% e4 = immse(simout.output.signals.values(:,2), real_data(:,6))

error = immse(simout.output.signals.values(mask,5), real_data(mask,2)) ...
      + immse(simout.output.signals.values(mask,4), real_data(mask,3)) ...
      + immse(simout.output.signals.values(mask,2), real_data(mask,6))/5000;

end