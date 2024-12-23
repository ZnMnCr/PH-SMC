function [value, isterminal, direction] = saturationEvent(x, kMax)

persistent wasAboveKMax

if isempty(wasAboveKMax)
    wasAboveKMax = false;
end

if x(67) > kMax && ~wasAboveKMax
    value = 0;  % 触发事件
    wasAboveKMax = true;
else
    value = 1;  % 不触发事件
end

isterminal = 1;  % 
direction = 0;   % 无方向限制，条件满足即可触发
end
%     % 检测条件
%     value = x(67) - kMax;      % 如果 x >= lambda_max，触发事件
%     isterminal = 1;              % 终止当前步并调整状态
%     direction = [];               % 
% 
% end