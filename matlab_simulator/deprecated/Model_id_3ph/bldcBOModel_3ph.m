function [xk1, y] = bldcBOModel_3ph(t, x, u, p, varargin)
    
    % p = p .* [1e3, 1e3, 1e1, 1e2, 1e-3]';


    p1 = p(1);
    p2 = p(2);
    p3 = p(3);
    p4 = p(4);
    % p5 = p(5);
    % p6 = p(6);

    u = u(:);
    x = x(:);

    % size(u)
    % size(x)


    % Parameters to estimate:
    R_Ls = p1;       % Rs/Ls
    inv_Ls = p2;     % 1/Ls
    lambda_Ls = p3;  % lambda_m/Ls
    P_lambda_J = p4; % (P * lambda_m) / J
    B_J = 0;        % B/J
    % theta0 = p5;     % initial angle 
    P=7;

    % Extract state variables
    % id = x(1);
    % iq = x(2);
    % omega = x(3);
    theta_e = x(4);

    bemf_a = trapezoidal(theta_e);
    bemf_b = trapezoidal(theta_e - 2/3*pi);
    bemf_c = trapezoidal(theta_e + 2/3*pi);

    Ts = 0.01;

    A = [1 - R_Ls*Ts,              0,                        0,                        -lambda_Ls * bemf_a * Ts, 0;
         0,                        1 - R_Ls*Ts,              0,                        -lambda_Ls * bemf_b * Ts, 0;
         0,                        0,                        1 - R_Ls*Ts,              -lambda_Ls * bemf_c * Ts, 0;
         P_lambda_J * bemf_a * Ts, P_lambda_J * bemf_b * Ts, P_lambda_J * bemf_c * Ts, 1,                        0;
         0,                        0,                        0,                        P*Ts,                        1];

    B = [inv_Ls, 0, 0;
         0, inv_Ls, 0;
         0, 0, inv_Ls;
         0, 0, 0;
         0, 0, 0];
    C = [1,0,0,0,0;
         0,1,0,0,0;
         0,0,1,0,0];
    D = zeros(3,3);

    xk1 = A*x + B*u;
    y = C * x + D * u;
end
