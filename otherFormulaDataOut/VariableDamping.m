

clc
clear all
close all
% Set Figure default values
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',2.0);
set(0,'DefaultAxesLineWidth',0.5);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesNextPlot','add')
% 设定eq的取值范围，这里以 -10到10为例，步长为0.1，可以根据实际需求调整
eq = -10:0.05:10;
% 给定K3一个固定值，例如设为2，你可以按需更改
K = 2;

% 按照公式计算K的值
 K1 = K + tanh(2*(log(1+abs(eq))).^2 - 0).^2*K;
K2 = K + tanh(2*(log(1+abs(eq))).^2 - 1).^2*K;
K3 = K + tanh(2*(log(1+abs(eq))).^2 - 5).^2*K;
 K4 = K + tanh(1*(log(1+abs(eq))).^2 - 2.5).^2*K;
K5 = K + tanh(2*(log(1+abs(eq))).^2 - 2.5).^2*K;
K6 = K + tanh(5*(log(1+abs(eq))).^2 - 2.5).^2*K;
% 绘制图像
plot(eq,K1,eq, K2,eq,K3,eq,K4,eq,K5,eq,K6);
xlabel('eq');
ylabel('K');
title('K vs eq');

% 保存数据到mat文件（保存变量eq和K到名为data的结构体中）
  save('VDa2b5.mat', 'eq', 'K1','K2','K3','K4','K5','K6' );

% 如果想保存图像为常见图片格式，比如png，可以使用如下命令
% print('image.png', '-dpng');


