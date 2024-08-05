function []=plotData(res)
% plot energy
fig1 = figure(1);
set(fig1, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
subplot(2,1,1)
plot(res.t,res.H)
xlabel('time (s)')
ylabel('Open-loop energy')
subplot(2,1,2)

plot(res.t,res.Hsmc)
grid on
xlabel('time (s)')
ylabel('Log of closed-loop energy $H_{smc}$')
saveas(fig1, ['Results/' '1.jpg']);
posArray=["x","y","z","\phi","\theta","\psi"];
% 创建六个子图
fig2 = figure(2);
set(fig2, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色
for i = 1:6
    subplot(3,2,i);
    hold on;
    % 绘制q和qd
    plot(res.t, res.q(:,i), colors{i}); 
    plot(res.t, res.qd(:,i), '--','Color',[0 0 0]);
    xlabel('Time (s)');
    ylabel('position');
   % 设置图例
    leg = legend(['${' char(posArray(i)) '}$'], ['${' char(posArray(i)) '}_d$']);
    set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
end
saveas(fig2, ['Results/' '2.jpg']);
% 创建六个子图
fig3 = figure(3);
set(fig3, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色
for i = 1:6
    subplot(3,2,i);
    hold on;
    
    plot(res.t,res.phi(:,i), colors{i}); 
    xlabel('Time (s)');
    ylabel('siliding mode surface');
   % 设置图例
    leg = legend(['${' char(posArray(i)) '}$'], 'Location', 'southeast');
    set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
end
sgtitle('The responses of siliding mode surface $\sigma$')
saveas(fig3, ['Results/' '3.jpg']);
% 创建六个子图
fig4 = figure(4);
set(fig4, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色
for i = 1:6
    subplot(3,2,i);
    hold on;
    
    plot(res.t,res.u(:,i), colors{i}); 
    xlabel('Time (s)');
    ylabel('Force (N)');
   % 设置图例
    leg = legend(['${' char(posArray(i)) '}$'] ,'Location', 'northeast');
    set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
end
sgtitle('The responses of input $u$')
saveas(fig4, ['Results/' '4.jpg']);
% 创建六个子图
fig5 = figure(5);
set(fig5, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色
for i = 1:6
    subplot(3,2,i);
    hold on;
    
    plot(res.t,res.qe(:,i), colors{i}); 
    xlabel('Time (s)');
    ylabel('error (m)');
   % 设置图例
    leg = legend(['${' char(posArray(i)) '}$'] ,'Location', 'southeast');
    set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
end
sgtitle('The responses of input $qe$')
saveas(fig5, ['Results/' '5.jpg']);


fig6=figure(6);
set(fig6, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色
for i = 1:6
    subplot(3,2,i);
    hold on;
    
    plot(res.t,res.match_distur(:,i), colors{i}); 
    xlabel('Time (s)');
    ylabel('Force (N)');
   % 设置图例
    leg = legend(['${' char(posArray(i)) '}$'] ,'Location', 'southeast');
    set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
end
sgtitle('The responses of disturbances')
saveas(fig6, ['Results/' '6.jpg']);