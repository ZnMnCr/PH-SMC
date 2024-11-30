clc
clear
close all
% 定义第二组三维点
x2 = [565, 465, 0, 0, -565, -465, 0, 0, 0];
y2 = [0, 0, -525, -425, 0, 0, 525, 425, 0];
z2 = [500, 500, 500, 500, 500, 500, 500, 500, 500];565

% 选择第一组点
x1_group = [x2(1), x2(3), x2(5), x2(7)];
y1_group = [y2(1), y2(3), y2(5), y2(7)];
z1_group = [z2(1), z2(3), z2(5), z2(7)];

% 连接首尾点
x1_group = [x1_group, x1_group(1)];
y1_group = [y1_group, y1_group(1)];
z1_group = [z1_group, z1_group(1)];

% 选择第二组点
x2_group = [x2(2), x2(4), x2(6), x2(8)];
y2_group = [y2(2), y2(4), y2(6), y2(8)];
z2_group = [z2(2), z2(4), z2(6), z2(8)];

% 连接首尾点
x2_group = [x2_group, x2_group(1)];
y2_group = [y2_group, y2_group(1)];
z2_group = [z2_group, z2_group(1)];

% 定义下平台的三维点
x1_down = [100, 0, -100, 0, 0, 20, 0, -20, 0];
y1_down = [0, -100, 0, 100, 20, 0, -20, 0, 0];
z1_down = [0, 0, 0, 0, 0, 0, 0, 0, 0];

% 选择下平台的第一组点
x1_down_group = [x1_down(1), x1_down(2), x1_down(3), x1_down(4)];
y1_down_group = [y1_down(1), y1_down(2), y1_down(3), y1_down(4)];
z1_down_group = [z1_down(1), z1_down(3), z1_down(3), z1_down(4)];

% 连接首尾点
x1_down_group = [x1_down_group, x1_down_group(1)];
y1_down_group = [y1_down_group, y1_down_group(1)];
z1_down_group = [z1_down_group, z1_down_group(1)];

% 选择下平台的第二组点
x2_down_group = [x1_down(5), x1_down(6), x1_down(7), x1_down(8)];
y2_down_group = [y1_down(5), y1_down(6), y1_down(7), y1_down(8)];
z2_down_group = [z1_down(5), z1_down(6), z1_down(7), z1_down(8)];

% 连接首尾点
x2_down_group = [x2_down_group, x2_down_group(1)];
y2_down_group = [y2_down_group, y2_down_group(1)];
z2_down_group = [z2_down_group, z2_down_group(1)];

% 创建一个新的图形窗口
figure;

% 绘制上平台的第一组点并用线连接
plot3(x1_group, y1_group, z1_group, '-o', 'Color', 'red', 'LineWidth', 2, 'DisplayName', 'Upper Platform Group 1');

% 绘制上平台的第二组点并用线连接
hold on;
plot3(x2_group, y2_group, z2_group, '-x', 'Color', 'blue', 'LineWidth', 2, 'DisplayName', 'Upper Platform Group 2');

% 绘制下平台的第一组点并用线连接
plot3(x1_down_group, y1_down_group, z1_down_group, '-s', 'Color', 'green', 'LineWidth', 2, 'DisplayName', 'Lower Platform Group 1');

% 绘制下平台的第二组点并用线连接
plot3(x2_down_group, y2_down_group, z2_down_group, '-d', 'Color', 'magenta', 'LineWidth', 2, 'DisplayName', 'Lower Platform Group 2');
% 增加以下部分用于在对应点之间连线
for i = 1:length(x1_group)
    % 在对应点之间连线
    plot3([x1_group(i), x1_down_group(i)], [y1_group(i), y1_down_group(i)], [z1_group(i), z1_down_group(i)], '-.', 'Color', 'black', 'LineWidth', 1);
end
plot3([x2_group(1), x2_down_group(2)], [y2_group(1), y2_down_group(2)], [z2_group(1), z2_down_group(2)], '-.', 'Color', 'black', 'LineWidth', 1);
plot3([x2_group(2), x2_down_group(3)], [y2_group(2), y2_down_group(3)], [z2_group(2), z2_down_group(3)], '-.', 'Color', 'black', 'LineWidth', 1);
plot3([x2_group(3), x2_down_group(4)], [y2_group(3), y2_down_group(4)], [z2_group(3), z2_down_group(4)], '-.', 'Color', 'black', 'LineWidth', 1);
plot3([x2_group(4), x2_down_group(1)], [y2_group(4), y2_down_group(1)], [z2_group(4), z2_down_group(1)], '-.', 'Color', 'black', 'LineWidth', 1);
plot3(x1_group, y1_group, z1_group, '-s', 'Color', 'green', 'LineWidth', 2, 'DisplayName', 'Lower Platform Group 1');
hold on;


% 选择下平台的点（假设前四个点构成了平台的四角）
x1_group = [x1_down(1), x1_down(2), x1_down(3), x1_down(4)];
y1_group = [y1_down(1), y1_down(2), y1_down(3), y1_down(4)];
z1_group = [z1_down(1), z1_down(2), z1_down(3), z1_down(4)];

% 连接首尾点形成闭合多边形
x1_group = [x1_group, x1_group(1)];
y1_group = [y1_group, y1_group(1)];
z1_group = [z1_group, z1_group(1)];

% 定义长方体的高度
height = 100; % 长方体的高度

% 定义长方体下平面的三维点
x_rect_bottom = x1_group(1:end-1);
y_rect_bottom = y1_group(1:end-1);
z_rect_bottom = z1_group(1:end-1);

% 计算长方体上平面的三维点
x_rect_top = x_rect_bottom;
y_rect_top = y_rect_bottom;
z_rect_top = z_rect_bottom - height;

% 绘制长方体的上下表面

plot3(x1_group, y1_group, z1_group, '-s', 'Color', 'green', 'LineWidth', 2, 'DisplayName', 'Bottom Surface');

hold on;
plot3(x_rect_top, y_rect_top, z_rect_top, '-s', 'Color', 'red', 'LineWidth', 2, 'DisplayName', 'Top Surface');

% 绘制长方体的侧面边缘
for i = 1:length(x_rect_bottom)
    plot3([x_rect_bottom(i), x_rect_top(i)], [y_rect_bottom(i), y_rect_top(i)], [z_rect_bottom(i), z_rect_top(i)], '-b', 'LineWidth', 2);
end

legend('show');
grid on;
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('Rectangular Prism Defined by Points');