clear
close all
clc


% fig_05 = openfig("ndp_long_H10H13\I13_2026-03-03--09-56-34_Tset0p5_OS0.fig", "new", "invisible");
% fig_1 = openfig("ndp_long_H10H13\I13_2026-03-03--09-57-50_Tset1_OS0.fig", "new", "invisible");
% fig_15 = openfig("ndp_long_H10H13\I13_2026-03-03--09-59-26_Tset1p5_OS0.fig", "new", "invisible");
% fig_3 = openfig("ndp_long_H10H13\I13_2026-03-03--10-17-23_Tset3_OS0.fig", "new", "invisible");


fig_05 = openfig("ndp_short_H10H13\I13_2026-03-03--10-03-01_Tset0p5_OS0.fig", "new", "invisible");
fig_1 = openfig("ndp_short_H10H13\I13_2026-03-03--10-02-21_Tset1_OS0.fig", "new", "invisible");
fig_15 = openfig("ndp_short_H10H13\I13_2026-03-03--10-01-29_Tset1p5_OS0.fig", "new", "invisible");
fig_3 = openfig("ndp_short_H10H13\I13_2026-03-03--10-18-31_Tset3_OS0.fig", "new", "invisible");


% fig_05 = openfig("ndp_random_correction_long_H10H13\I13_2026-03-03--10-04-34_Tset0p5_OS0.fig", "new", "invisible");
% fig_1 = openfig("ndp_random_correction_long_H10H13\I13_2026-03-03--10-05-28_Tset1_OS0.fig", "new", "invisible");
% fig_15 = openfig("ndp_random_correction_long_H10H13\I13_2026-03-03--10-06-43_Tset1p5_OS0.fig", "new", "invisible");
% fig_3 = openfig("ndp_random_correction_long_H10H13\I13_2026-03-03--10-07-30_Tset3_OS0.fig", "new", "invisible");

% fig_05 = openfig("ndp_random_correction_short_H10H13\I13_2026-03-03--10-15-33_Tset0p5_OS0.fig", "new", "invisible");
% fig_1 = openfig("ndp_random_correction_short_H10H13\I13_2026-03-03--10-14-01_Tset1_OS0.fig", "new", "invisible");
% fig_15 = openfig("ndp_random_correction_short_H10H13\I13_2026-03-03--10-10-53_Tset1p5_OS0.fig", "new", "invisible");
% fig_3 = openfig("ndp_random_correction_short_H10H13\I13_2026-03-03--10-09-30_Tset3_OS0.fig", "new", "invisible");




allAxes = findall(fig_05, 'type', 'axes');
ax = allAxes(end);

allLines = findall(ax, 'type', 'line');

ref = allLines(1);  % 1 ref 2 vel

x_ref = get(ref, 'XData');
y_ref = get(ref, 'YData');


omega_05 = allLines(2);  % 1 ref 2 vel
x_05 = get(omega_05, 'XData');
y_05 = get(omega_05, 'YData');



allAxes = findall(fig_1, 'type', 'axes');
ax = allAxes(end);
allLines = findall(ax, 'type', 'line');
omega_1 = allLines(2);  % 1 ref 2 vel
x_1 = get(omega_1, 'XData');
y_1 = get(omega_1, 'YData');



allAxes = findall(fig_15, 'type', 'axes');
ax = allAxes(end);
allLines = findall(ax, 'type', 'line');
omega_15 = allLines(2);  % 1 ref 2 vel
x_15 = get(omega_15, 'XData');
y_15 = get(omega_15, 'YData');



allAxes = findall(fig_3, 'type', 'axes');
ax = allAxes(end);
allLines = findall(ax, 'type', 'line');
omega_3 = allLines(2);  % 1 ref 2 vel
x_3 = get(omega_3, 'XData');
y_3 = get(omega_3, 'YData');

figure(Position=[100,100,400,250])
title("model 'short'")% with distribution correction")
hold on
plot(x_05, y_05, 'DisplayName',"T_{set}=0.5", 'LineWidth',1)
plot(x_1, y_1, 'DisplayName',"T_{set}=1", 'LineWidth',1)
plot(x_15, y_15, 'DisplayName',"T_{set}=1.5", 'LineWidth',1)
plot(x_3, y_3, 'DisplayName',"T_{set}=3", 'LineWidth',1)
plot(x_ref, y_ref, 'DisplayName',"Reference", 'LineStyle','--', 'Color','k')
ylim([0,2500])
xlabel("Time [s]")
ylabel("Speed [rpm]")

legend('Location','southeast')