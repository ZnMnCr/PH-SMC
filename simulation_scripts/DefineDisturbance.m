function distur=DefineDisturbance(t)

    mu = 0; % 设定均值
    sigma = 3; % 设定标准差
        % 定义匹配干扰
t_start_disturbance = 0; % 干扰开始的时间
t_end_disturbance = 4; % 干扰开始的时间
    Goss = mu + sigma*randn(6, 1); % 每次调用都会重新生成高斯白噪声
    disturbance_amplitude = [600000; 700000; 700000; 10000; 70000; 0]*0;
    distur = disturbance_amplitude*(2*sin(10*t)+1*sin(90*t)+1*sin(223*t)) * (t >= t_start_disturbance && t <=t_end_disturbance)+0.01*disturbance_amplitude.*Goss;
end