function [sys]=DefineDisturbance()
% 定义匹配干扰
t_start_disturbance = 3; % 干扰开始的时间
disturbance_amplitude = [1000000;700000;700000;1000000;0;0]*0; % 干扰的幅值
sys.match_distur = @(t) disturbance_amplitude *sin(t)* (t >= t_start_disturbance);