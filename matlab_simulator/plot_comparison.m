clc
clear
close all



list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end

inertia = 15;

alpha = 0.6;


color_tr = [     0 0.4196 0.6510, alpha];
color_bo = [0.8663 0.4061 0.2384, alpha];
color_vrft = [0.4846 0.7497 0.5805, alpha];





fprintf("inertia %d\n", inertia)



switch inertia
    case 13


        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H13/I13_2025-11-11--11-13-39.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H13/I13_2025-11-11--11-13-39_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_13__ki-0.0000-kp-28.8122/I_13__2025-11-27--10-04-17.csv");
        tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_13__ki-0.0000-kp-2.8007/I_13__2025-11-27--12-17-56.csv");


    case 5


        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H05/I05_2025-11-11--11-41-35.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H05/I05_2025-11-11--11-41-35_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_05__ki-0.0152-kp-15.9433/I_05__2025-11-27--14-44-53.csv");
        % tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_");
    
    case 15


        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H15/I15_2025-11-11--12-10-34.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H15/I15_2025-11-11--12-10-34_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_15__ki-0.0000-kp-11.9834/I_15__2025-11-27--14-48-46.csv");
        tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_15__ki-0.0000-kp-2.9085/I_15__2025-11-27--14-54-14.csv");

    case 9


        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H09/I09_2025-11-11--14-42-05.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H09/I09_2025-11-11--14-42-05_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_09__ki-0.1000-kp-50.0000/I_09__2025-11-27--15-22-39.csv");
        tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_09__ki-0.0000-kp-3.8716/I_09__2025-11-27--15-21-10.csv");

    case 11

        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H11/I11_2025-11-11--15-10-53.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H11/I11_2025-11-11--15-10-53_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_11__ki-0.0000-kp-43.1595/kp_0.0000_ki_43.1595.csv");
        tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_11__ki-0.0000-kp-3.8905/I_11__2025-11-27--15-36-43.csv");

    case 7

        tab_tr = readtable("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H07/I07_2025-11-11--15-31-17.csv");
        metadata = load("../data/transformer_v2_CAN_exp/new_dataset_short_noise_h10_30k_H10H07/I07_2025-11-11--15-31-17_metadata.mat");
        
        
        tab_bo = readtable("../data/CL_experiments_benchmark_BO/inertia_07__ki-0.0000-kp-50.0000/I_07__2025-11-27--15-30-24.csv");
        tab_vrft = readtable("../data/CL_experiments_benchmark_VRFT/inertia_07__ki-0.0000-kp-2.8185/I_07__2025-11-27--15-28-58.csv");

end














% r = tab_tr.r;
omega = tab_tr.omega;

% start_idx = max(1, find(r>1800,1,"first") - 1);
start_idx = max(1, find(omega>50,1,"first") - 4);
end_idx = start_idx + 500;


omega_cut = tab_tr.omega(start_idx:end_idx);
% vd_cut = tab_tr.vd(start_idx:end_idx);
% vq_cut = tab_tr.vq(start_idx:end_idx);
% id_cut = tab_tr.id(start_idx:end_idx);
iq_cut = tab_tr.iq(start_idx:end_idx);
% iq_ref_cut = tab_tr.iq_ref(start_idx:end_idx);
% r_cut = tab_tr.r(start_idx:end_idx);
t_cut = tab_tr.t(start_idx:end_idx);
t_cut = t_cut - t_cut(1);
Ts = t_cut(2)-t_cut(1);


OS_percent = metadata.metadata.OS;
T_set = metadata.metadata.T_s;

s = tf('s');
z = tf('z',Ts);
z.Variable = 'z^-1';

OS = OS_percent / 100;
zeta = -log(OS) / sqrt(pi^2 + (log(OS))^2);
wn = 3 / (zeta * T_set);
num = [wn^2];
den = [1, 2*zeta*wn, wn^2];
fprintf("Overshoot: %f%%\n", OS_percent)
fprintf("Settling time: %fs\n", T_set)



M = tf(num, den);
M = c2d(M,Ts,'tustin');



[y_model, t_model] = step(2000*M, 0:Ts:5);

try
    iq_ref = tab_bo.iq_ref;
    start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
    end_idx = start_idx + 5000;

catch
    iq = tab_bo.iq;
    start_idx = max(1, find(iq>0.1,1,"first") - 1);
    end_idx = start_idx + 500;
end


omega_cut_bo = tab_bo.omega(start_idx:end_idx);
iq_cut_bo = tab_bo.iq(start_idx:end_idx);
time_bo = tab_bo.t(start_idx:end_idx) - tab_bo.t(start_idx);

try
    iq_ref = tab_vrft.iq_ref;
    start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
    end_idx = start_idx + 5000;
    
    omega_cut_vrft = tab_vrft.omega(start_idx:end_idx);
    iq_cut_vrft = tab_vrft.iq(start_idx:end_idx);
    time_vrft = tab_vrft.t(start_idx:end_idx) - tab_vrft.t(start_idx);
catch
    fprintf("No vrft found\n")
end





figure;
hold on
plot(t_cut, omega_cut, 'Color',color_tr, 'LineWidth',1, 'DisplayName', '$\mathcal{C}$')
plot(t_model, y_model, '--', 'Color', 'k' , 'LineWidth',2, 'DisplayName', '$M$')
try
plot(time_vrft, omega_cut_vrft, 'Color',color_vrft, 'LineWidth',1, 'DisplayName', 'VRFT')
catch
end
plot(time_bo, omega_cut_bo, 'Color',color_bo, 'LineWidth',1, 'DisplayName', 'BO')
tit = sprintf('Step Response ($T_{set}\\approx %.2fs, OS_{\\%%}\\approx %.2f\\%%$)', T_set, OS_percent);
title(tit, 'Interpreter','latex');
grid on;
xlim([0,1.5])
ylim([0,2500])
legend('Location','southeast')





figure('Position',[200,200,500,300])
subplot(211)
hold on
box on
plot(t_model, y_model, '--', 'Color','k', 'LineWidth',2, 'DisplayName', '$M$')
try
plot(time_vrft, omega_cut_vrft, 'Color',color_vrft, 'LineWidth',2, 'DisplayName', 'VRFT')
catch
end
plot(time_bo, omega_cut_bo, 'Color',color_bo, 'LineWidth',2, 'DisplayName', 'BO')
plot(t_cut, omega_cut, 'Color',color_tr, 'LineWidth',2, 'DisplayName', '$\mathcal{C}$')
tit = sprintf('Step Response ($T_{set}\\approx %.2fs, OS_{\\%%}\\approx %.2f\\%%$)', T_set, OS_percent);
title(tit, 'Interpreter','latex');
ylabel("$\omega$  [rpm]")
xlim([0,1.5])
ylim([0,2500])
legend('Location','southeast')

subplot(212)
hold on
box on
try
plot(time_vrft, iq_cut_vrft, 'Color',color_vrft, 'LineWidth',2, 'DisplayName', 'VRFT')
catch
end
plot(time_bo, iq_cut_bo, 'Color',color_bo, 'LineWidth',2, 'DisplayName', 'BO')
plot(t_cut, iq_cut, 'Color',color_tr, 'LineWidth',2, 'DisplayName', '$\mathcal{C}$')
ylabel("$i_q$ [A]")
xlabel("Time [s]")
xlim([0,1.5])
% ylim([0,2500])
legend()




fig = figure('Position',[200,200,1000,300]);
tiledlayout(2,6)

for i = 1:6









end


