clear
clc
close all


temp_name = strsplit(pwd,'in-context-bldc');
user_tmp = strsplit(pwd,'Users\');
user_tmp2 = strsplit(user_tmp{2},'\');
user = user_tmp2{1};



folder_list = ["50_percent_control_v2", "50_percent_control_v2_disturbed", "50_percent_control_v2_perturbed"];

datapath_list = string(length(folder_list));

for folder_idx = 1:length(folder_list)
    datapath_tmp = fullfile(temp_name{1}, 'in-context-bldc', 'data', 'simulated_v2', folder_list(folder_idx));
    datapath_list(folder_idx) = datapath_tmp;
    addpath(datapath_tmp)
end


dataset_distribution.R = [];
dataset_distribution.L = [];
dataset_distribution.Flux = [];
dataset_distribution.Im = [];
dataset_distribution.B = [];
dataset_distribution.Id = [];
dataset_distribution.i_omega = [];


for datapath = datapath_list

    file_list = dir(sprintf('%s',fullfile(datapath, "metadata")));
    del_log_list = [];
    
    for i = 1:length(file_list)
        if strcmp(file_list(i).name,'desktop.ini') | strcmp(file_list(i).name,'.') | strcmp(file_list(i).name,'..') | strcmp(file_list(i).name,'.DS_Store' )     % 'desktop.ini' is a hidden file created by google drive in each directory. 
            del_log_list(end+1) = i;                                                                                    % It is deleted from the list since it is not useful for the program.
        end
    end
    
    file_list(del_log_list) = [];
    total_exp = length(file_list);

    for i = 1:total_exp
    
        file = file_list(i);
        data = load(fullfile(datapath,"metadata",file.name));
        dataset_distribution.R       = [dataset_distribution.R, data.BLDC.StatorPhaseResistance];
        dataset_distribution.L       = [dataset_distribution.L, data.BLDC.InductanceLd];
        dataset_distribution.Flux    = [dataset_distribution.Flux, data.BLDC.FluxLinkage];
        dataset_distribution.Im      = [dataset_distribution.Im, data.BLDC.Inertia];
        dataset_distribution.B       = [dataset_distribution.B, data.BLDC.ViscousFrictionCoefficient];
        dataset_distribution.Id      = [dataset_distribution.Id, data.disc.Inertia];
        dataset_distribution.i_omega = [dataset_distribution.i_omega, data.i_omega];



    end

end


save("dataset_distribution.mat", "dataset_distribution")



model_name = 'new_dataset_short_noise_scaled_h10_40k_H10H.mat';
savepath_tmp = "C:\Users\" + user + "\OneDrive - Politecnico di Milano\in-context-bldc-data\simulated";
folder_name = sprintf('statistical_analysis_model_%s', model_name(1:end-4));

datapath = fullfile(savepath_tmp, folder_name);
addpath(datapath)

addpath(datapath)

file_list = dir(sprintf('%s',datapath));

del_log_list = [];

for i = 1:length(file_list)
    if strcmp(file_list(i).name,'desktop.ini') | strcmp(file_list(i).name,'.') | strcmp(file_list(i).name,'..') | strcmp(file_list(i).name,'.DS_Store' ) | ~contains(file_list(i).name, 'params')      % 'desktop.ini' is a hidden file created by google drive in each directory. 
        del_log_list(end+1) = i;                                                                                    % It is deleted from the list since it is not useful for the program.
    end
end

file_list(del_log_list) = [];

testset_distribution.R = [];
testset_distribution.L = [];
testset_distribution.Flux = [];
testset_distribution.Im = [];
testset_distribution.B = [];
testset_distribution.Id = [];
testset_distribution.i_omega = [];

total_exp = length(file_list);

for i = 1:total_exp

    file = file_list(i);
    data = load(fullfile(datapath,file.name));
    testset_distribution.R       = [testset_distribution.R, data.BLDC.StatorPhaseResistance];
    testset_distribution.L       = [testset_distribution.L, data.BLDC.InductanceLd];
    testset_distribution.Flux    = [testset_distribution.Flux, data.BLDC.FluxLinkage];
    testset_distribution.Im      = [testset_distribution.Im, data.BLDC.Inertia];
    testset_distribution.B       = [testset_distribution.B, data.BLDC.ViscousFrictionCoefficient];
    testset_distribution.Id      = [testset_distribution.Id, data.disc.Inertia];
    testset_distribution.i_omega = [testset_distribution.i_omega, data.i_omega];

end


save("testset_distribution.mat", "testset_distribution")




