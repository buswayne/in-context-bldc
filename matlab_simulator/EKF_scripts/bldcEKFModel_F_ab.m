function xk1 = bldcEKFModel_F_ab(x, u, Rs, Ls, Kt, J, Ts)
    

    u = u(:);
    x = x(:);


    R_Ls = Rs/Ls;            % Rs/Ls
    inv_Ls = 1/Ls;           % 1/Ls
    lambda_Ls = Kt/Ls;       % lambda_m/Ls
    P_lambda_J = 3/2*7*Kt/J; % (3/2 * P * lambda_m) / J
    B_J = 0;                 % B/J
    P=7;

    % Extract state variables
    % id = x(1);
    % iq = x(2);
    omega = x(3);
    theta_e = x(4);

    % Continuous-time state-space matrices
    A = [ -R_Ls, omega, 0, 0;
         -omega, -R_Ls, -lambda_Ls, 0;
          0, P_lambda_J, -B_J, 0;
          0, 0, P, 0];

    B = [ inv_Ls * cos(theta_e), inv_Ls * sin(theta_e);
         -inv_Ls * sin(theta_e), inv_Ls * cos(theta_e);
          0, 0;
          0, 0];
    

    % C = [ 1, 0, 0, 0;
    %       0, 1, 0, 0];
    % 
    % D = [0, 0; 0, 0]; % No direct feedthrough

    % Discretization using Zero-Order Hold (ZOH)
    % Ts = 0.01;
    % A_d = expm(A * Ts);
    % B_d = A \ (A_d - eye(size(A))) * B; % Numerical integral
    % C_d = C;  % No change in C
    % D_d = D;  % No change in D
    % Ts = 0.01;
    % sys_cont = ss(A, B, C, D); % Create continuous-time state-space model
    % sys_disc = c2d(sys_cont, Ts, 'zoh'); % Discretize using ZOH
    % A_d = sys_disc.A;
    % B_d = sys_disc.B;
    % C_d = sys_disc.C;
    % D_d = sys_disc.D;

    dx = A*x + B*u;
    xk1 = x + dx*Ts;
end
