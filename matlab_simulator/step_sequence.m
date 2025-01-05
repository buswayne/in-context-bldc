function sequence = step_sequence(T, Ts, min_val, max_val, min_duration, max_duration)
%Reference generation
%   Detailed explanation goes here
total_samples = T/Ts;
sequence = zeros(total_samples,1);
i = 1;
while i < total_samples
    remaining_samples = total_samples - i;
    % step_duration_samples = randi([min_duration / Ts, max_duration / Ts + 1]);
    step_duration_samples = round(min_duration + (max_duration-min_duration)*rand()/Ts);
    step_value = min_val + (max_val - min_val) * rand();
    step_duration = min(step_duration_samples, remaining_samples);
    sequence(i:i+step_duration) = step_value;
    i = i + step_duration;
end

end