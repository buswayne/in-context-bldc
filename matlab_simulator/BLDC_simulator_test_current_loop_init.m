clear
clc
close all

%%% starts the BLDC simulator multiple times to test different combinations
%%% of Kp and Ki for the current PI controller

tic
now_string = string(datetime('now'),"yyyy-MM-dd_HH-mm-ss");

speed_loop = 0;
current_loop = 1;

set_parameters
T = 2;
Ts = 1e-4;
time = 0:Ts:T-Ts;

Max_abs_value = BLDC.CurrentMax;
Min_duration = 0.1;
Max_duration = 0.3;

reference_current = step_sequence_current(T, Ts,Max_abs_value, Min_duration, Max_duration);

P_list = [1];
I_list = [200];
for P = P_list
    for I = I_list
        PID_current.p = P;
        PID_current.i = I;
        
        
        
        speed_input.time = time;
        speed_input.signals.values = zeros(length(time),1);
        load_input.time = time;
        load_input.signals.values = zeros(length(time),1);
        current_input.time = time;
        current_input.signals.values = reference_current;
        voltage_d_input.time = time;
        voltage_d_input.signals.values = zeros(length(time),1);
        voltage_q_input.time = time;
        voltage_q_input.signals.values = zeros(length(time),1);

        mdl = 'BLDC_simulator';
        
        output = sim(mdl);
        
        
        output_clean.t = output.output.time;
        output_clean.theta = output.output.signals.values(:,1);
        output_clean.omega = output.output.signals.values(:,2);
        output_clean.r = output.output.signals.values(:,3);
        output_clean.i_d = output.output.signals.values(:,4);
        output_clean.i_q = output.output.signals.values(:,5);
        output_clean.i_q_ref = output.output.signals.values(:,6);
        output_clean.v_d = output.output.signals.values(:,7);
        output_clean.v_q = output.output.signals.values(:,8);
        
        out_tab = struct2table(output_clean);
        
        exp_name = "Experiment_" + now_string + ".csv";
        toc
        
        figure
        ax1 = subplot(3,1,1);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,3), "DisplayName","Omega ref")
        plot(output.output.time, output.output.signals.values(:,2), "DisplayName","Omega")
        legend(["Omega ref", "Omega"])
        tit = "P: " + P + ", I: " + I;
        title(tit)

        ax2 = subplot(3,1,2);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,6), "DisplayName","iq ref")
        plot(output.output.time, output.output.signals.values(:,5), "DisplayName","iq")
        plot(output.output.time, output.output.signals.values(:,4), "DisplayName","id")
        legend()
        
        ax3 = subplot(3,1,3);
        hold on
        grid on
        plot(output.output.time, output.output.signals.values(:,7), "DisplayName","vd")
        plot(output.output.time, output.output.signals.values(:,8), "DisplayName","vq")
        legend()
        linkaxes([ax1, ax2, ax3], 'x')
    end
end

