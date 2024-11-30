function [value, isterminal, direction] = saturationEvent(x, kMax)
    % 检测条件
    value = x(67) - kMax;      % 如果 x >= lambda_max，触发事件
    isterminal = 1;              % 终止当前步并调整状态
    direction = 1;               % 单向检测（x 变大时触发）
    
end
