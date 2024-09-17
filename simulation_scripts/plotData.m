function []=plotData(res,ctrl)
% Set Figure default values
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',2.0);
set(0,'DefaultAxesLineWidth',0.5);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesNextPlot','add')
% plot energy
fig1 = figure(1);
set(fig1, 'Position', [100 100 1000 800]); % 第三个和第四个参数分别是宽度和高度
subplot(2,1,1)
plot(res.t,res.H)
xlabel('time (s)')
ylabel('Open-loop energy')
subplot(2,1,2)

plot(res.t,res.Hd)
grid on
xlabel('time (s)')
ylabel('Closed-loop energy $H_{smc}$')
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
% 创建一个新的图形窗口
if ctrl.selector == 1
fig3 = figure(3);
set(fig3, 'Position', [100 100 1000 800]); % 设置图形窗口的位置和大小

% 定义颜色序列
colors = {'b', 'g', 'r', 'c', 'm', 'y'}; % 不同的颜色

clf;

% 绘制每一条曲线
for i = 1:6
    plot(res.t, res.phi(:,i), colors{i}); % 使用指定的颜色绘制曲线
    hold on; % 允许在同一图上绘制多条线
end

% 设置图表属性
xlabel('Time (s)');
ylabel('Sliding mode surface');
title('The responses of sliding mode surface $\sigma$');

% 设置图例
legends = cell(1, 6);
for i = 1:6
    legends{i} = ['$' char(posArray(i)) '$']; % 假设 posArray 已经定义好
end
leg = legend(legends, 'Location', 'best');
set(leg, 'Interpreter', 'latex'); % 设置LaTeX解释器
% zp = BaseZoom();
% zp.run;
%  zp.run;
% 保存图形
saveas(fig3, ['Results/' '3.jpg']);
 end
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
    % if i == 1
    % zp = BaseZoom();
    %  zp.run;
    % end
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


fig7=figure(7);
% 假设 res.q 和 res.qd 是 Nx6 的数组
% 实际轨迹和姿态数据
xyz_actual = res.q(:, 1:3);


% 理想轨迹和姿态数据
xyz_ideal = res.qd(:, 1:3);


% 提取 x, y, z 坐标
x_actual = xyz_actual(:, 1);
y_actual = xyz_actual(:, 2);
z_actual = xyz_actual(:, 3);

x_ideal = xyz_ideal(:, 1);
y_ideal = xyz_ideal(:, 2);
z_ideal = xyz_ideal(:, 3);




% 绘制三维轨迹
plot3(x_actual, y_actual, z_actual, 'r');
hold on;
plot3(x_ideal, y_ideal, z_ideal,'--','Color',[0 0 0]);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3)

legend("q", "$q_d$");
hold off;
