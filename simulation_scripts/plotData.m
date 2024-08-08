function []=plotData(res)
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

plot(res.t,res.Hsmc)
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


fig7=figure(7);
% 假设 res.q 和 res.qd 是 Nx6 的数组
% 实际轨迹和姿态数据
xyz_actual = res.q(:, 1:3);
eulerAngles_actual = res.q(:, 4:6);

% 理想轨迹和姿态数据
xyz_ideal = res.qd(:, 1:3);
eulerAngles_ideal = res.qd(:, 4:6);

% 提取 x, y, z 坐标
x_actual = xyz_actual(:, 1);
y_actual = xyz_actual(:, 2);
z_actual = xyz_actual(:, 3);

x_ideal = xyz_ideal(:, 1);
y_ideal = xyz_ideal(:, 2);
z_ideal = xyz_ideal(:, 3);

% % 定义箭头的长度
% arrowLength = 0.1;
% 
% % 预分配方向矢量数组
% u_actual = zeros(size(x_actual));
% v_actual = zeros(size(y_actual));
% w_actual = zeros(size(z_actual));
% 
% u_ideal = zeros(size(x_ideal));
% v_ideal = zeros(size(y_ideal));
% w_ideal = zeros(size(z_ideal));
% 
% % 计算实际轨迹的方向矢量
% for i = 1:length(x_actual)
%     % 获取当前点的欧拉角
%     roll = eulerAngles_actual(i, 1);
%     pitch = eulerAngles_actual(i, 2);
%     yaw = eulerAngles_actual(i, 3);
% 
%     % 将欧拉角转换为方向矢量
%     direction = eul2rotm([yaw, pitch, roll]) * [1; 0; 0]; % 假设 x 轴是前进方向
%     u_actual(i) = direction(1) * arrowLength;
%     v_actual(i) = direction(2) * arrowLength;
%     w_actual(i) = direction(3) * arrowLength;
% end
% 
% % 计算理想轨迹的方向矢量
% for i = 1:length(x_ideal)
%     % 获取当前点的欧拉角
%     roll = eulerAngles_ideal(i, 1);
%     pitch = eulerAngles_ideal(i, 2);
%     yaw = eulerAngles_ideal(i, 3);
% 
%     % 将欧拉角转换为方向矢量
%     direction = eul2rotm([yaw, pitch, roll]) * [1; 0; 0]; % 假设 x 轴是前进方向
%     u_ideal(i) = direction(1) * arrowLength;
%     v_ideal(i) = direction(2) * arrowLength;
%     w_ideal(i) = direction(3) * arrowLength;
% end

% 绘制三维轨迹
plot3(x_actual, y_actual, z_actual, 'r');
hold on;
plot3(x_ideal, y_ideal, z_ideal,'--','Color',[0 0 0]);
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(3)
% 绘制姿态箭头
% quiver3(x_actual, y_actual, z_actual, u_actual, v_actual, w_actual, 'r');
% quiver3(x_ideal, y_ideal, z_ideal, u_ideal, v_ideal, w_ideal, 'k');
legend("q", "$q_d$");
hold off;
