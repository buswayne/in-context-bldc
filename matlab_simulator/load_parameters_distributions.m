clear
clc
close all


rng("default")
perturbation = 0.5;
coeff = @(x) (1 - perturbation) + (perturbation * 2) * x;
coeff_inertia = @(x) (0.1) + (9.9) * x;

theorical_distribution.R = 0.994 * 0.357256158228637  * coeff(rand(50000,1));
theorical_distribution.L = 0.995e-3 * 1.383264744587776  * coeff(rand(50000,1));
poles = 7;
RatedSpeed = 4390 / 30 * pi * coeff(rand(50000,1));
theorical_distribution.Flux = 48./RatedSpeed/(poles) * 1.179953752195608;
theorical_distribution.Im = 44e-07  * coeff(rand(50000,1));
theorical_distribution.B = 0.0083*1e-6  * coeff(rand(50000,1));
theorical_distribution.Id = (8.7749e-04 - 44e-07) * coeff_inertia(rand(50000,1));
theorical_distribution.i_omega = 2.497  * coeff(rand(50000,1));




load("dataset_distribution.mat")
load("testset_distribution.mat")
color_th = "#edae49";
color_ds = "#d1495b";
color_ts = "#00798c";
alpha_hist = 0.5;





variable = 'R';
figure
hold on 

histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Motor resistance [\Omega]")
ylabel("Density")


% 
% figure
% hold on
% histogram(theorical_distribution.R, 'Normalization','pdf', 'NumBins',20)
% histogram(dataset_distribution.R, 'Normalization','pdf', 'NumBins',20)
% histogram(testset_distribution.R, 'Normalization','pdf', 'NumBins',20)
% 
% variable = 'R';
% [y_th,x_th] = kde(theorical_distribution.(variable));
% [y_ds,x_ds] = kde(dataset_distribution.(variable));
% [y_ts,x_ts] = kde(testset_distribution.(variable));
% figure
% hold on
% plot(x_th, y_th,"DisplayName","Theoretical distribution")
% plot(x_ds, y_ds,"DisplayName","Dataset distribution")
% plot(x_ts, y_ts,"DisplayName","Test set distribution")
% yl = ylim;
% ylim([yl(1), yl(2)*1.2])
% legend()
% xlabel("Motor resistance [\Omega]")
% ylabel("Density")


variable = 'L';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Motor inductance [H]")
ylabel("Density")



variable = 'Flux';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Motor Flux [Wb]")
ylabel("Density")



variable = 'Im';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Motor inertia [Kg*m^2]")
ylabel("Density")


variable = 'B';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Motor friction [{N*m*s}/{rad}]")
ylabel("Density")



variable = 'Id';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Disc Inertia [Kg*m^2]")
ylabel("Density")


variable = 'i_omega';
figure
hold on 
histogram(theorical_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Theoretical distribution", "FaceColor",color_th, "FaceAlpha",alpha_hist)
histogram(dataset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Dataset distribution", "FaceColor", color_ds, "FaceAlpha",alpha_hist)
histogram(testset_distribution.(variable), 'Normalization','pdf', 'NumBins',20,"DisplayName","Test set distribution", "FaceColor", color_ts, "FaceAlpha",alpha_hist)

[y_th,x_th] = kde(theorical_distribution.(variable));
[y_ds,x_ds] = kde(dataset_distribution.(variable));
[y_ts,x_ts] = kde(testset_distribution.(variable));
plot(x_th, y_th, 'Color', color_th,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ds, y_ds, 'Color', color_ds,"HandleVisibility",'off', 'LineWidth',2)
plot(x_ts, y_ts, 'Color', color_ts,"HandleVisibility",'off', 'LineWidth',2)

yl = ylim;
ylim([yl(1), yl(2)*1.2])
legend()
xlabel("Simulator speed gain [-]")
ylabel("Density")