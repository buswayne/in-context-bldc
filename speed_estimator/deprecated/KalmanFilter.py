import numpy as np


#KALMAN FILTER FUNCTION DEFINITION

def kf_predict(F, Q, H, x_prev, P_prev):
    # Perform prediction step
    x_t = F @ x_prev

    # Predict output
    y_t = H @ x_t

    P_t = F @ P_prev @ F.T + Q * (H > 0)

    return x_t, y_t, P_t

def kf_update(H, R, z_t, x_t, P_t):
    # Perform update step
    S_t = H @ P_t @ H.T + R
    K_t = P_t @ H.T @ np.linalg.inv(S_t)
    
    # Update state estimate
    x_t = x_t + K_t @ (z_t - H @ x_t)

    # x_t = np.maximum(x_t, 0)

    P_t = (np.eye(x_t.shape[0]) - K_t @ H) @ P_t

    return x_t, P_t, K_t

def my_kf(x_0, P_0, F, Q, R, output_matrix, measurements):
    states = []     # x hat
    outputs = []    # y hat
    cov_matrix = []  # P
    k_s = []

    for t in range(output_matrix.shape[0]):
        H = output_matrix[t:t+1, :]
        if t == 0:
            states.append(x_0)
            cov_matrix.append(P_0)

        # Prediction step
        x_t, y_t, P_t = kf_predict(F=F, Q=Q, H=H, x_prev=states[t], P_prev=cov_matrix[t])
        # Update step
        x_t, P_t, K_t = kf_update(H, R, measurements[t], x_t, P_t)
        
        # Append results
        states.append(x_t)
        outputs.append(y_t[0])
        cov_matrix.append(P_t)
        k_s.append(K_t[:, 0])

    states = np.array(states)
    outputs = np.array(outputs)
    
    return states, outputs, np.array(cov_matrix), np.array(k_s)

def my_var_kf(x_0, P_0, F, Q, R, output_matrix, measurements,x_hat):
    states = []     # x hat
    outputs = []    # y hat
    cov_matrix = []  # P
    k_s = []

    for t in range(output_matrix.shape[0]):
        H = output_matrix[t:t+1, :]
        if t == 0:
            states.append(x_0)
            cov_matrix.append(P_0)

        # Prediction step
        x_t, y_t, P_t = kf_predict(F=F, Q=Q, H=H, x_prev=states[t], P_prev=cov_matrix[t])

        # Update step
        x_t, P_t, K_t = kf_update(H, R, measurements[t], x_t, P_t)

        # Compute the lower confidence bound
        var_x_pred = x_hat[t] - 2 * np.sqrt(x_t)  

        # Adjust variances where the lower bound is negative
        for i in range(len(x_t)):
            if var_x_pred[i] < 0:  # Lower bound is invalid
                x_t[i] = (x_hat[t][i] / 2) ** 2  # Set minimum variance to satisfy the constraint
        
        # Append results
        states.append(x_t)
        outputs.append(y_t[0])
        cov_matrix.append(P_t)
        k_s.append(K_t[:, 0])

    states = np.array(states)
    outputs = np.array(outputs)

    # # Optional clipping for non-negative states
    # states[states < 0] = 0
    # outputs[outputs < 0] = 0
    
    return states, outputs, np.array(cov_matrix), np.array(k_s)

def my_akf(x_0,P_0,F,Q,Rs,output_matrix,measurements):
    states = []     #x hat
    outputs = []    # y hat
    cov_matrix = []  # P

    for t in range(output_matrix.shape[0]):
        H = output_matrix[t:t+1,:]
        R = Rs[t]
        if t == 0:
            states.append(x_0)
            cov_matrix.append(P_0)

        x_t,y_t,P_t = kf_predict(F=F,Q=Q,H=H,x_prev=states[t],P_prev=cov_matrix[t])
        x_t,P_t,K_t = kf_update(H,R,measurements[t],x_t,P_t)

        states.append(x_t)    
        outputs.append(y_t[0])
        cov_matrix.append(P_t)
    
    return np.array(states),np.array(outputs),np.array(cov_matrix)

