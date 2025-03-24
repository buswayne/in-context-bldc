clear
clc
close all
tic



temp_name = strsplit(pwd,'in-context-bldc');
datapath = fullfile(temp_name{1}, "in-context-bldc","data","CL_experiments_double_sensor\train\inertia13_ki-0.0061-kp-11.8427");


files = dir(fullfile(datapath, '*.csv')); % List all CSV files

U_all = cell(length(files), 1); % Preallocate for inputs
Y_all = cell(length(files), 1); % Preallocate for outputs
Ts = 0.01; % Sampling time

for k = 1:length(files)
    filePath = fullfile(datapath, files(k).name);
    data = readtable(filePath);

    % Extract relevant columns
    V_alpha = data.va;
    V_beta = data.vb;
    I_alpha = data.ia;
    I_beta = data.ib;
    omega = data.omega;
    theta_e = data.theta_e;

    % Store data as a new experiment
    
    U_all{k} = [V_alpha, V_beta];  % Control inputs (each row: [V_alpha, V_beta])
    Y_all{k} = [I_alpha, I_beta];  % Measured outputs
    % size(U_all{k})
    % size(Y_all{k})
end

% Create multi-experiment iddata object
data_id = iddata(Y_all{1}, U_all{1}, Ts, 'Name', 'BLDC');

% Define initial parameter guesses
p0.Name = ['p1','p2','p3','p4','p5']';
p0.Unit = '';
% p0.Value = [250, 700, 10, 200, 0.001]'; % Initial guesses for Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J, B/J
p0.Value = [100, 1000, 50, 100, 0.005]'; % Initial guesses for Rs/Ls, 1/Ls, lambda_m/Ls, (3/2 * P * lambda_m) / J, B/J
% scale
p0.Value = p0.Value ./ [1e3, 1e3, 1e1, 1e2, 1e-3]';
p0.Minimum = [1e-5,1e-5,1e-5,1e-5,1e-5]';
p0.Maximum = [1e5,1e5,1e5,1e5,1e5]';
p0.Fixed = [false,false,false,false,false]';

% Create the IDGREY model for discrete-time system
order = [2, 2, 4]; % [Number of outputs, number of inputs, number of states]
% m = idnlgrey(@(t, x, u, p) bldcGreyBoxModel(t, x, u, p, Ts), order, p0, [0;0;0;0], Ts);
m = idnlgrey(@bldcGreyBoxModel, order, p0, [0;0;0;0], Ts);

% Allow different initial conditions for each experiment
% opt = nlgreyestOptions('Display', 'on', 'SearchMethod', 'lsqnonlin');
opt = nlgreyestOptions('Display', 'on', 'SearchMethod', 'lsqnonlin');
opt.SearchOptions.MaxIterations = 100; % Limit iterations for debugging
opt.SearchOptions.TolFun = 1e-8; % Set tolerance

% Estimate model using all experiments
sys_est = nlgreyest(data_id, m, opt);

% Extract estimated parameters
p_est = sys_est.Parameters.Value;
p_est = p_est .* [1e3, 1e3, 1e1, 1e2, 1e-3]';
Rs_Ls = p_est(1);
inv_Ls = p_est(2);
lambda_Ls = p_est(3);
P_lambda_J = p_est(4);
B_J = p_est(5);
P=7;
% Compute final motor parameters
Ls = 1 / inv_Ls;
Rs = Rs_Ls * Ls;
lambda_m = lambda_Ls * Ls;
J = (3/2 * P * lambda_m) / P_lambda_J;
B = B_J * J;

% Display results
disp("Estimated Parameters:");
disp(["Rs = " num2str(Rs)]);
disp(["Ls = " num2str(Ls)]);
disp(["Lambda_m = " num2str(lambda_m)]);
disp(["J = " num2str(J)]);
disp(["B = " num2str(B)]);

% Validate model on a few experiments
for k = 1:min(3, length(Y_all))  % Plot first 3 experiments
    Time = (0:length(Y_all{k})-1) * Ts;
    Y_sim = sim(sys_est, U_all{k}', Time);

    figure;
    subplot(2,1,1);
    plot(Time, Y_all{k}(:,1), 'b', Time, Y_sim(:,1), 'r--');
    title(['Experiment ' num2str(k) ': I_\alpha Validation']);
    legend('Measured', 'Simulated');
    
    subplot(2,1,2);
    plot(Time, Y_all{k}(:,2), 'g', Time, Y_sim(:,2), 'm--');
    legend('Measured', 'Simulated');
    xlabel('Time (s)');
    ylabel('Current (A)');
end




