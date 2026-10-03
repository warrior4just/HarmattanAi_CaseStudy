function [soc_est, v1_est, P_history] = ekf_soc_estimator(time, current, voltage, param)
    % EKF_SOC_ESTIMATOR Estimates SoC and v1 using Polynomial curve fitting.
    %
    % INPUTS:
    %   time    : Vector of timestamps (seconds) [N x 1]
    %   current : Vector of measured cell current (Amperes) [N x 1] (Positive=Charge, Negative=Discharge)
    %   voltage : Vector of measured terminal voltage (Volts) [N x 1]
    %   param   : Structure containing cell parameters (R0, R1, C1, Q_4S1P, soc_init, Q_cov, R_cov)

    N = length(time);
    
    %% Output Arrays
    soc_est = zeros(N, 1);
    v1_est = zeros(N, 1);
    P_history = zeros(2, 2, N);
    
    %% Initialization
    % State vector: x = [v1; soc]
    x = [0.0; param.soc_init]; 
    
    % Initial error covariance matrix
    P = [10, 0; 
         0,    0.05]; 
     
    v1_est(1) = x(1);
    soc_est(1) = x(2);
    P_history(:, :, 1) = P;
    
    % Convert nominal capacity from Ah to Ampere-seconds (Coulombs)
    Q_sec = param.Q_4S1P * 3600;
    
    %% Define 5th-Order Polynomial Coefficients
    % Obtained from a 3.7V Nominal Lithium (NMC/LiHV) drone cell curve fit
    % OCV = p1*soc^5 + p2*soc^4 + p3*soc^3 + p4*soc^2 + p5*soc + p6
    p = [7.034, -18.156, 16.302, -5.922, 1.942, 3.001]; 
    
    % Unpack vector terms cleanly for explicit derivative tracking
    p1 = p(1); p2 = p(2); p3 = p(3); p4 = p(4); p5 = p(5); p6 = p(6);

    %% Main Recursive loop
    for k = 2:N
        dt = time(k) - time(k-1);
        ik = current(k);
        vk_meas = voltage(k);
        
        % Zero-Order Hold discretization term for the RC network
        tau = param.R1 * param.C1;
        exp_term = exp(-dt / tau);

        % PREDICTION (Time Update)
        % Propagate state variables forward
        v1_minus = exp_term * x(1) + param.R1 * (1 - exp_term) * ik;
        soc_minus = x(2) + (dt / Q_sec) * ik;
        soc_minus = max(0.0, min(1.0, soc_minus)); % Keep bounded within physical limits
        
        x_minus = [v1_minus; soc_minus];
        
        % Linear State Transition Jacobian matrix (A_k)
        A = [exp_term, 0;
             0,        1];
         
        % Propagate State Error Covariance matrix forward
        P_minus = A * P * A' + param.Q_cov;
        
        % CORRECTION (Measurement Update)
        % Capture the baseline state variable scalar to pass to equations
        s = x_minus(2); 
        
        % Explicit 5th-Order Polynomial evaluation for forward OCV
        ocv_pred = p1*s^5 + p2*s^4 + p3*s^3 + p4*s^2 + p5*s + p6;
        
        % EXPLICIT CALCULATION OF THE DERIVATIVE: d(OCV)/d(SoC)
        % Evaluates the exact slope equation algebraically
        d_ocv_d_soc = 5*p1*s^4 + 4*p2*s^3 + 3*p3*s^2 + 2*p4*s + p5;
        
        % Predict terminal cell voltage output 
        vk_pred = ocv_pred + x_minus(1) + (param.R0 * ik);
        
        % Calculate Innovation Residual (Voltage sensor error)
        y_tilde = vk_meas - vk_pred;
        
        % Formulate Measurement Jacobian matrix (C_k) using explicit derivative scalar
        C = [1, d_ocv_d_soc];
        
        % Compute the Kalman Gain
        S = C * P_minus * C' + param.R_cov; % Innovation covariance
        K = (P_minus * C') / S;             % Kalman gain vector [2 x 1]
        
        % Update State Array with corrected weights
        x = x_minus + K * y_tilde;
        x(2) = max(0.0, min(1.0, x(2)));    % Secondary physical bound clamp post-correction
        
        % Update State Error Covariance Matrix (Joseph Form for high numerical stability)
        I = eye(2);
        P = (I - K * C) * P_minus * (I - K * C)' + K * param.R_cov * K';
        
        % STORE RESULTS
        
        v1_est(k) = x(1);
        soc_est(k) = x(2);
        P_history(:, :, k) = P;
    end
end
