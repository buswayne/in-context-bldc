clc
clear
close all 

list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'latex');
end
% "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\BO_data\inertia07_tmp_experiments_0DT\kp_0.0000_ki_29.8864.csv"
% "C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\BO_data\inertia05_tmp_experiments_0DT\kp_0.0001_ki_50.0000.csv"


% tab_C1PI_on_C6 = readtable("C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\BO_data\inertia07_tmp_experiments_0DT\kp_0.0000_ki_29.8864.csv");
% tab_C1PI_on_C6 = readtable("C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\BO_data\inertia07_tmp_experiments_0DT\kp_0.1000_ki_48.7747.csv");
% 
% tab_C6PI_on_C1 = readtable("C:\Users\39340\OneDrive - Politecnico di Milano\in-context-bldc-data\BO_data\inertia05_tmp_experiments_0DT\kp_0.0001_ki_50.0000.csv");

tab_C1PI_on_C6 = readtable("../data/Bad_PI/inertia_07__ki-0.0152-kp-15.9400/I_07__2025-12-05--15-02-11.csv");

tab_C6PI_on_C1 = readtable("../data/Bad_PI/inertia_05__ki-0.0000-kp-100.0000/I_05__2025-12-05--14-49-09.csv");


iq_ref = tab_C1PI_on_C6.iq_ref;
start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
end_idx = start_idx + 5000;


omega_cut_C1PI_on_C6 = tab_C1PI_on_C6.omega(start_idx:end_idx);
iq_cut_C1PI_on_C6 = tab_C1PI_on_C6.iq(start_idx:end_idx);
t_cut_C1PI_on_C6 = tab_C1PI_on_C6.t(start_idx:end_idx) - tab_C1PI_on_C6.t(start_idx);


iq_ref = tab_C6PI_on_C1.iq_ref;
start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
end_idx = start_idx + 5000;

omega_cut_C6PI_on_C1 = tab_C6PI_on_C1.omega(start_idx:end_idx);
iq_cut_C6PI_on_C1 = tab_C6PI_on_C1.iq(start_idx:end_idx);
t_cut_C6PI_on_C1 = tab_C6PI_on_C1.t(start_idx:end_idx) - tab_C6PI_on_C1.t(start_idx);



% fig = figure('Position',[100,100,500,200]);
% hold on
% box on
% 
% plot(t_cut_C1PI_on_C6, omega_cut_C1PI_on_C6)
% plot(t_cut_C6PI_on_C1, omega_cut_C6PI_on_C1)
% 
% fig = figure('Position',[100,100,500,200]);
% hold on
% box on
% 
% plot(t_cut_C1PI_on_C6, iq_cut_C1PI_on_C6)
% plot(t_cut_C6PI_on_C1, iq_cut_C6PI_on_C1)


%%%%


tab_C1PI_on_C1 = readtable("../data/CL_experiments_benchmark_BO/inertia_05__ki-0.0152-kp-15.9433/I_05__2025-11-27--14-44-53.csv");

tab_C6PI_on_C6 = readtable("../data/CL_experiments_benchmark_BO/inertia_07__ki-0.0000-kp-50.0000/I_07__2025-11-27--15-30-24.csv");



iq_ref = tab_C6PI_on_C6.iq_ref;
start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
end_idx = start_idx + 5000;

omega_cut_C6PI_on_C6 = tab_C6PI_on_C6.omega(start_idx:end_idx);
iq_cut_C6PI_on_C6 = tab_C6PI_on_C6.iq(start_idx:end_idx);
t_cut_C6PI_on_C6 = tab_C6PI_on_C6.t(start_idx:end_idx) - tab_C6PI_on_C6.t(start_idx);



iq_ref = tab_C1PI_on_C1.iq_ref;
start_idx = max(1, find(iq_ref>0.1,1,"first") - 1);
end_idx = start_idx + 5000;

omega_cut_C1PI_on_C1 = tab_C1PI_on_C1.omega(start_idx:end_idx);
iq_cut_C1PI_on_C1 = tab_C1PI_on_C1.iq(start_idx:end_idx);
t_cut_C1PI_on_C1 = tab_C1PI_on_C1.t(start_idx:end_idx) - tab_C1PI_on_C1.t(start_idx);



% color_C1 = '#9d4edd';
% color_C6 = '#248277';
color_C1 = '#00adc4';
color_C6 = '#e5b07a';
lw = 1.25;

color_C1_tmp = color_shades_saturation(cell2mat(hex2rgb(color_C1)), 3, 0.1);
color_C1 = lighten_color(color_C1_tmp{end}, 0.1);



fig = figure('Position',[100,100,500,200]);

% sp = subplot(2,1,1);
% hold on
% 
% L1 = plot(nan,nan, 'Color',color_C1, 'DisplayName',"PI tuned on $S^{(1)}$");
% L2 = plot(nan,nan, 'Color',color_C6, 'DisplayName',"PI tuned on $S^{(6)}$");
% legend([L1,L2], 'Location','northoutside')
% sp.Visible = "off";
% sp.Legend.Visible = "on";





tl = tiledlayout(1,2, "TileSpacing","tight");
tl.OuterPosition = [0, 0, 1, 0.9];
xlabel(tl, "Time [s]", 'Interpreter', 'latex', 'fontsize', 10);
ylabel(tl, "$\omega$ [rpm]", 'Interpreter', 'latex', 'fontsize', 10);


nexttile
hold on 
box on
% xlabel("Time [s]")
% ylabel("$\omega$ [rpm]")

p2 = plot(t_cut_C6PI_on_C1, omega_cut_C6PI_on_C1, 'Color', color_C6, 'LineWidth',lw+0.5);
p1 = plot(t_cut_C1PI_on_C1, omega_cut_C1PI_on_C1, 'Color', color_C1, 'LineWidth',lw-0.1);
xlim([0,0.5])
ylim([0,2500])
% drawnow
ax1 = gca;
ax1.XTick(end+1) = 0.5;
ax1.XTickLabel{end} = [ax1.XTickLabel{end}, '\,\,\,\,'];
% L = legend('Location','southeast');
% L.IconColumnWidth = 15;
% L.AutoUpdate = "off";


x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [0, 0, 1900, 1900];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)

x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [2100, 2100, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)


x_corners = [-0, 1.5, 1.5, -0]; 
y_corners = [2400, 2400, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)



str = sprintf("$S^{(%d)}$", 1);
text(0.93, 0.12, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'w', ... 
    'EdgeColor', 'k', ...
    'FontSize', 10, ...
    'Color','w');
text(0.93, 0.1, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'none', ... 
    'EdgeColor', 'none', ...
    'Color', 'k', ...
    'FontSize', 10);





nexttile
hold on 
box on
% xlabel("Time [s]")

plot(t_cut_C6PI_on_C6, omega_cut_C6PI_on_C6, 'Color', color_C6, 'DisplayName', 'PI tuned on $S^{(6)}$', 'LineWidth',lw+0.5);
plot(t_cut_C1PI_on_C6, omega_cut_C1PI_on_C6, 'Color', color_C1, 'DisplayName', 'PI tuned on $S^{(1)}$', 'LineWidth',lw-0.1);
xlim([0,2])
ylim([0,2500])
yticklabels([])
ax2 = gca;
% ax2.XTickLabel{1} = ['\,\,', ax2.XTickLabel{1}];
% L = legend('Location','southeast');
% L.IconColumnWidth = 15;
% L.AutoUpdate = "off";



x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [0, 0, 1900, 1900];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)

x_corners = [1.5, 5, 5, 1.5]; 
y_corners = [2100, 2100, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)


x_corners = [-0, 1.5, 1.5, -0]; 
y_corners = [2400, 2400, 3000, 3000];
p = patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
patch(x_corners, y_corners, 'r', 'FaceColor', '#adb5bd', 'EdgeColor', 'none', 'FaceAlpha', 0.1);
hh = hatchfill(p, 'single', 45, 5);
set(hh, 'Color', '#adb5bd', 'LineWidth', 0.5)



str = sprintf("$S^{(%d)}$", 6);
text(0.93, 0.12, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'w', ... 
    'EdgeColor', 'k', ...
    'FontSize', 10, ...
    'Color','w');
text(0.93, 0.1, str, ...
    'Units', 'normalized', ... 
    'HorizontalAlignment', 'right', ... 
    'VerticalAlignment', 'bottom', ... 
    'BackgroundColor', 'none', ... 
    'EdgeColor', 'none', ...
    'Color', 'k', ...
    'FontSize', 10);


lgd = legend([findobj(gca, 'DisplayName', 'PI tuned on $S^{(1)}$'), ...
              findobj(gca, 'DisplayName', 'PI tuned on $S^{(6)}$')], 'NumColumns',2); 
lgd.IconColumnWidth = 15;

% lgd.Layout.Tile = 'North';
% tl.Padding = 'tight';
lgd.Position(1) = 0.5 - lgd.Position(3)/2; % Center horizontally
lgd.Position(2) = 0.95; % Adjust vertical height manually (0 to 1 scale)



savefig("figs_paper/bad_PI.fig")
saveas(gcf, "figs_paper/bad_PI.png")
fig = gcf;
set(fig, 'PaperPositionMode', 'auto');
exportgraphics(fig, 'figs_paper/bad_PI.pdf', 'ContentType', 'vector');
