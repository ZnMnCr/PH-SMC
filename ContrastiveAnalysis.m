addpath('./simulation_scripts');
posArray=["x","y","z","$\phi$","$\theta$","$\psi$"];

set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',2.0);
set(0,'DefaultAxesLineWidth',0.5);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesNextPlot','add')
% 定义颜色和线型
colors = { [112,176,218]/256, ... 
           [131,53,142]/256, ...
           [220,97,44]/256, ... % 绿色
           [0.9290, 0.6940, 0.1250], ... % 黄色
           [0.4940, 0.1840, 0.5560], ... % 紫色
           [0.4660, 0.6740, 0.1880] }; % 另一种绿色
lineStyles = {'-', '--', '-.', ':'};
% 创建一个新的图形窗口
% 假设文件名分别为 file1.mat, file2.mat, file3.mat
filenames = {'TSMC.mat', 'sakata.mat', 'Joel.mat'};

for i = 1:length(filenames)
    load(fullfile('./Results', filenames{i}));
end
fig9 = figure(9);
set(fig9, 'Position', [100 100 1000 800]);

% 遍历每个结构体
structNames = {'TSMC', 'sakata', 'Joel'};
for k = 1:length(structNames)
    currentStruct = eval(structNames{k});
    
    % 对于每一个误差类型
    for i = 1:6
        subplot(3,2,i);
        hold on;
        
        % 绘制当前结构体的误差数据
          plot(currentStruct.t, currentStruct.qe(:,i), ...
            [lineStyles{k}], ... % 使用不同的线型来区分结构体
             'DisplayName', sprintf('%s - %s', structNames{k}, posArray{i}), ...
             'Color', colors{k}); % 使用RGB颜色
    %             if i == 1 && k == 3
    % zp = BaseZoom();
    %  zp.run;
    % end
        % 设置x轴和y轴标签
        xlabel('Time (s)');
        ylabel('error (m)');
        
        % 设置图例
   
            leg = legend('Location', 'southeast');
            set(leg, 'Interpreter', 'latex');
       grid on  
    end
   
end

% 设置子图的标题
sgtitle('The responses of input $qe$', 'Interpreter', 'latex');

% 保存图表
saveas(fig9, fullfile('Results', '9.jpg'));

