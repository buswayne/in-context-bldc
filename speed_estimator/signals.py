import numpy as np
import matplotlib.pyplot as plt

def steps_sequence(T, Ts, min_val, max_val, min_duration, max_duration):
    # Calculate the total number of samples
    total_samples = int(T / Ts)

    # Initialize array for the final sequence
    steps_sequence = np.empty(total_samples)

    current_index = 0

    while current_index < total_samples:
        # Determine remaining samples
        remaining_samples = total_samples - current_index

        # Randomly determine the duration for the current step in samples
        step_duration_samples = np.random.randint(int(min_duration / Ts), int(max_duration / Ts) + 1)

        # Ensure we don't exceed the remaining samples
        step_duration = min(step_duration_samples, remaining_samples)

        # Generate a random step value between min_val and max_val
        step_value = np.random.uniform(min_val, max_val)

        # Fill the corresponding part of the sequence with the step value
        steps_sequence[current_index:current_index + step_duration] = step_value

        # Update the current index
        current_index += step_duration

    return steps_sequence.reshape(-1, 1)
