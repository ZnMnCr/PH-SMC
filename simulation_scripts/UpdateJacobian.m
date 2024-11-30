function [J] = UpdateJacobian(x)

% 下平台连接点坐标 (x1, y1, z1)
x1 = [100, 0, -100, 0, 0, 20, 0, -20]*0.1;
y1 = [0, -100, 0, 100, 20, 0, -20, 0]*0.1;
%%%%-----------这里坐标系改到下了
z1 = [0, 0, 0, 0, 0, 0, 0, 0];
DD = [x1; y1; z1;ones(1,8)];
% 上平台连接点坐标 (x2, y2, z2)

x2 = [565, 465,  0 ,  0 ,-565, -465,  0,   0 ]*0.1;
y2 = [ 0 ,  0 ,-525,-425,  0 ,   0 , 525, 425 ]*0.1;
z2 = [500, 500, 500, 500, 500,  500, 500, 500]*0.1;
% 下平台点到上平台点的对应关系
index_lower = [1, 2, 3, 4, 5, 6, 7, 8];
index_upper = [1, 3, 5, 7, 8, 2, 4, 6];

% 初始化雅克比矩阵 (包含线速度和力矩部分)
num_cables = length(x1);
J = zeros(6, num_cables);  % 6行表示x, y, z分量和力矩分量
% 计算下平台到上平台的变换矩阵
Txyz = [1, 0, 0, x(1);
        0, 1, 0,  x(2);
        0, 0, 1,  x(3); 
         0, 0, 0  ,1]; 
% 绕x轴翻转指定角度

R_x = [  1, 0,          0,         0;
         0, cos(x(4)), -sin(x(4)), 0; 
         0, sin(x(4)), cos(x(4)),  0
         0, 0,          0,         1]; 

% 绕y轴翻转指定角度】

R_y = [cos(x(5)),  0, sin(x(5)), 0;  
       0,          1, 0,         0; 
       -sin(x(5)), 0, cos(x(5)), 0;
        0,         0, 0,         1];


    % 对下平台点进行齐次变换
    D = Txyz *R_x*R_y* DD;
   % 平台中心坐标 (假设平台中心在下平台上，z1的平均值)
   platform_center = [mean(D(1,:)), mean(D(2,:)), mean(D(3,:))];
    % 计算每根绳索的方向余弦和力矩分量
for i = 1:num_cables

% 获取当前绳索的下平台和上平台连接点
    lower_idx = index_lower(i);
    upper_idx = index_upper(i);
    
    % 计算绳索方向向量
    Lx = x2(upper_idx) - D(1,lower_idx);
    Ly = y2(upper_idx) - D(2,lower_idx);
    Lz = z2(upper_idx) - D(3,lower_idx);
    CableVector = [Lx; Ly; Lz];
    % 计算绳索长度
    L_length = norm([Lx; Ly; Lz]);
    
    % 计算单位方向向量 (线速度雅克比矩阵部分)
    J(1:3, i) = CableVector / L_length;
    
    % 计算力矩分量 (力矩雅克比矩阵部分)
    b_vector = [D(1,lower_idx) - platform_center(1); 
                D(2,lower_idx) - platform_center(2); 
                D(3,lower_idx) - platform_center(3)];
    J(4:6, i) = cross(b_vector, J(1:3, i));
end

