function [sys]=DefineDisturbance(sys)
% 定义匹配干扰
t_start_disturbance = 0; % 干扰开始的时间
disturbance_amplitude = [1000000;700000;700000;1000000;700000;0]*0.08; % 干扰的幅值
t_end_disturbance = 10; % 干扰结束的时间
sys.match_distur = @(t) disturbance_amplitude*(2*sin(t)+sin(10*t)) * (t >= t_start_disturbance && t <=t_end_disturbance);