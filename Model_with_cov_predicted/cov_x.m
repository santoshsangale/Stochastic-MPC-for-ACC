function sigma_x = cov_x(N,sig)
 % covariance matrix propagation
    T_s=0.2;
    w_cov = [sig^2 0; 0 T_s^2*sig^2];
    
    A=[1 0; -T_s 1];
    %B=[T_s 0; -0.5*T_s^2 0];

    Bw= [1 0; 0 1];

    %K = [0.2858 -0.4910];
    %phi = A-B*K;

    % initial covariance matrix
    sigma_x = zeros(2,2,N);
    sigma_x(:,:,1)=0; % is 0 matrix, i.e., '1' refers to current state, which is known

    % updated covariance matrix for each step
    for i = 2:N
        sigma_x(:,:,i) = A*sigma_x(:,:,i-1)*A' + Bw*w_cov*Bw';
    end
    
end

