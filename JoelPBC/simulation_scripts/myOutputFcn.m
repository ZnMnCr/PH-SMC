% 迭代输出函数
function status = myOutputFcn(t, y, flag)
    if strcmp(flag, 'init')
        fprintf('Simulation started.\n');
    elseif strcmp(flag, '')
        fprintf('At time %.2f \n', t);
    elseif strcmp(flag, 'done')
        fprintf('Simulation finished.\n');
    end
    status = [];
end