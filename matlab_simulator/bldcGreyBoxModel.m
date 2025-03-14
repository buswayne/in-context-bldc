function [dx, y] = bldcGreyBoxModel(t, x, u, p, varargin)
    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    p4 = p(4);
    p5 = p(5);

    u = u(:);


    % Parameters to estimate:
    R_Ls = p1;       % Rs/Ls
    inv_Ls = p2;     % 1/Ls
    lambda_Ls = p3;  % lambda_m/Ls
    P_lambda_J = p4; % (3/2 * P * lambda_m) / J
    B_J = p5;        % B/J
    P=7;

    % Extract state variables
    % id = x(1);
    % iq = x(2);
    omega = x(3);
    theta_e = x(4);

    % Continuous-time state-space matrices
    A = [ -R_Ls, omega, 0, 0;
         -omega, -R_Ls, lambda_Ls, 0;
          0, P_lambda_J, -B_J, 0;
          0, 0, P, 0];

    B = [ inv_Ls * cos(theta_e), inv_Ls * sin(theta_e);
         -inv_Ls * sin(theta_e), inv_Ls * cos(theta_e);
          0, 0;
          0, 0];

    C = [ cos(theta_e), -sin(theta_e), 0, 0;
          sin(theta_e),  cos(theta_e), 0, 0];

    D = [0, 0; 0, 0]; % No direct feedthrough

    % Discretization using Zero-Order Hold (ZOH)
    % Ts = 0.01;
    % A_d = expm(A * Ts);
    % B_d = A \ (A_d - eye(size(A))) * B; % Numerical integral
    % C_d = C;  % No change in C
    % D_d = D;  % No change in D
    Ts = 0.01;
    sys_cont = ss(A, B, C, D); % Create continuous-time state-space model
    sys_disc = c2d(sys_cont, Ts, 'zoh'); % Discretize using ZOH
    A_d = sys_disc.A;
    B_d = sys_disc.B;
    C_d = sys_disc.C;
    D_d = sys_disc.D;

    dx = A_d*x + B_d*u;
    y = C_d * x + D_d * u;
end
