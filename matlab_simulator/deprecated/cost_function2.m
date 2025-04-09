function [error] = cost_function2(var)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

temp_name = strsplit(pwd,'in-context-bldc');

real_data_path = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments\train\inertia13_ki-0.0061-kp-11.8427\2024-10-16--10-57-42_exp  26.csv");
real_data = readmatrix(real_data_path);
real_data = real_data(1:200,:);
% mask = real_data(:,2) >= 1;
mask = (real_data(:,1) >= 0.2 & real_data(:,1) <= 0.4) | (real_data(:,1) >= 1.2 & real_data(:,1) <= 1.5);

i_r = var.i_r;
i_l = var.i_l;
i_f = var.i_f;
i_i = var.i_i;
i_bv = var.i_b;
i_a = var.i_a;
% i_p = var.i_p;

mdl = 'BLDC_simulator_BO';

simIn = Simulink.SimulationInput(mdl);
simIn = setVariable(simIn,'i_r', i_r);
simIn = setVariable(simIn,'i_l', i_l);
% simIn = setVariable(simIn,'i_f', i_f*7/i_p);
simIn = setVariable(simIn,'i_f', i_f);
simIn = setVariable(simIn,'i_i', i_i);
simIn = setVariable(simIn,'i_bv', i_bv);
simIn = setVariable(simIn,'i_a', i_a);
% simIn = setVariable(simIn,'BLDC.PolePairs', i_p);
simout = sim(simIn);

% sim(mdl)

error = immse(simout.output.signals.values(mask,5), real_data(mask,2)) ...
      + immse(simout.output.signals.values(mask,4), real_data(mask,3)) ...
      + immse(simout.output.signals.values(:,2), real_data(:,6))/50000;

end