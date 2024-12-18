function [error] = cost_function(var)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

temp_name = strsplit(pwd,'in-context-bldc');

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:200,:);
mask = real_data(:,2) >= 1;

i_r = var.i_r;
i_l = var.i_l;
i_f = var.i_f;
i_i = var.i_i;
i_bv = var.i_b;
i_a = var.i_a;

mdl = 'BLDC_simulator_BO';

simIn = Simulink.SimulationInput(mdl);
simIn = setVariable(simIn,'i_r', i_r);
simIn = setVariable(simIn,'i_l', i_l);
simIn = setVariable(simIn,'i_f', i_f);
simIn = setVariable(simIn,'i_i', i_i);
simIn = setVariable(simIn,'i_bv', i_bv);
simIn = setVariable(simIn,'i_a', i_a);
simout = sim(simIn);

% sim(mdl)

error = immse(simout.output.signals.values(mask,5), real_data(mask,2)) + immse(simout.output.signals.values(mask,4), real_data(mask,3));

error = error^2;
end