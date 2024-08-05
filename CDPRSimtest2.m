clear
close all
clc

addpath('./simulation_scripts');
% Set Figure default values
set(0,'DefaultTextInterpreter','latex');
set(0,'DefaultLegendInterpreter','latex');
set(0,'DefaultAxesFontSize',11);
set(0,'DefaultLineLineWidth',2.0);
set(0,'DefaultAxesLineWidth',0.5);
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
set(0,'defaultAxesNextPlot','add')

%% Simulation settings
% Simulation step 仿真步长
sim.delta_t = 0.01;
% Simulation length仿真时长
sim.t_end = 20;
pa.m=6000;%动平台质量
pa.g=[0;0;9.8];%重力加速度
%% Define 6DOF CDPR
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';


[sys.M,sys.P]=massMatrix(pa.m,pa.g);
sys.V=sys.P;
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sum(sys.V(q));
%sys.D= @(q) diag([5,2,3,1,2,1]);
sys.D= @(q) diag([0.1,0.1,0.1,0.1,0.1,0.1]);
sys.G = @(q) diag([1;1;1;1;1;1]);
sys.Hdq = matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[{q_sym}, {p_sym}]);
sys.Hdp =  matlabFunction(jacobian(sys.H(q_sym,p_sym),p_sym).','vars',[{q_sym}, {p_sym}]);
sys.dVdq = matlabFunction(jacobian(sum(sys.V(q_sym)),q_sym).','vars',{q_sym});

% 定义匹配干扰
t_start_disturbance = 3; % 干扰开始的时间
disturbance_amplitude = [1000000;2000000;700000;150;0;0]*0.5; % 干扰的幅值
match_distur = @(t) disturbance_amplitude *sin(t)* (t >= t_start_disturbance);
dx = @(q,p,u,t) [zeros(6) eye(6); -eye(6) -sys.D(q)]*[sys.Hdq(q,p); sys.Hdp(q,p)] + [zeros(6); sys.G(q)]*u+ [zeros(6); sys.G(q)]*match_distur(t);
ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
ctrl.D = @(q) sys.D(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);
% Define target trajectory and derivatives
%定义轨迹
ctrl.qd = @(t) [10*(1/2)*cos(t)+5*(1/2)*sin(2*t); 10*(1/2)*cos(t);10*(1/2)*cos(t)+5*(1/2)*sin(3*t);0;0;0];
ctrl.dqd = @(t) [-10*(1/2)*sin(t)+10*(1/2)*cos(2*t); -10*(1/2)*sin(t);-10*(1/2)*sin(t)+5*0.5*3*cos(3*t);0;0;0];
ctrl.ddqd = @(t) [-10*(1/2)*cos(t)-20*(1/2)*sin(2*t); -10*(1/2)*cos(t);-10*(1/2)*cos(t)-5*0.5*3*3*sin(3*t);0;0;0];
% Compute the target momentum from (13)
%期望动量的坐标变换
ctrl.pd = @(t,q) ctrl.T(q)\ctrl.dqd(t);

% Compute the error coordinates on q, p from (11), (15)
%速度和动量误差
ctrl.ep = @(t,q,p) p - ctrl.pd(t,q);
ctrl.eq = @(t,q) q - ctrl.qd(t);
% Compute the gradient of the reference momentum with respect to
% configuration as per (16)
%求期望动量的梯度
ctrl.dpddq = matlabFunction(jacobian(ctrl.pd(t_sym,q_sym),q_sym),'vars',[{t_sym}, {q_sym}]);
%% Define Passivity-based sliding mode controller
%VI. NUMERICAL EXAMPLE Case1 and K
 %K=tril(ones(6));
 K=diag([1000;1000;1000;1;1;1]);

 phi=@(t,q,p) (K*ctrl.eq(t,q)+ctrl.ep(t,q,p));%Here q is a variable to be determined
%phi=@(q,p) K*q+tan(p);
%Take a partial derivative of \phi
dphideq=matlabFunction(jacobian(phi(t_sym,q_sym,p_sym),q_sym),'vars',{q_sym});
dphidep=matlabFunction(jacobian(phi(t_sym,q_sym,p_sym),p_sym),'vars',{p_sym});
%Replace the variable to be determined q with eq(\tilde \q)

%compute the Lambda from eq.(22)
he=sym(1000*0.5*(dphideq(q_sym)*ctrl.T(q_sym))*dphidep(p_sym)');%这里的系数对系统收敛到滑模面上有影响
%he=100000*0.5*(dphidq(q_sym)*ctrl.T(q_sym))*dphidep(t_sym,q_sym,p_sym)';
Lambda=matlabFunction(2*(he+he'),'vars',[{p_sym}]);

%Compute the partial derivative of U
normPhi=phi(t_sym,q_sym,p_sym);
dUdPhi=matlabFunction((phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);
%Then the feedback controller from eq.(23)
ctrl.v=@(t,q,p) (-inv(dphidep(p))*Lambda(p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(p))*dphideq(q))*ctrl.T(q))*ctrl.ep(t,q,p));
%ctrl.v=@(t,q,p) (-inv(dphidep(t,q,p))*Lambda(t,q,p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(t,q,p))*dphidq())*ctrl.T(q))*ctrl.ep(t,q,p));
%ctrl.v=@(t,q,p) zeros(6,1);
%input u from eq.(20)
ctrl.u = @(t,q,p) ctrl.G(q)\(ctrl.D(q)*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.T(q)*p) + ctrl.T(q)\ctrl.ddqd(t) + ctrl.T(q)'*sys.dVdq(q) + ctrl.v(t,q,p));

% Define  closed-loop energy in eq.(24)
ctrl.KE = @(t,q,p) 0.5*sum(ctrl.ep(t,q,p).^2);%kinetic energy
ctrl.U=matlabFunction((sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);%potential energy
ctrl.Hsmc = @(t,q,p) ctrl.KE(t,q,p) + ctrl.U(t,q,p);



%% Run simulation
% Define initial conditions
sim.q0 = [0 0 0 0 0 0].';
sim.p0 = [0 0 0 0 0 0].';
sim.x0 = [sim.q0; sim.p0];
options = odeset('OutputFcn', @myOutputFcn,'RelTol',0.2e-2);

% Comcatinate model with control law
ode = @(t,x) dx(x(1:6),x(7:12),ctrl.u(t,x(1:6),ctrl.p(x(1:6),x(7:12))),t);
% Solve ODE
[res.t,res.x] = ode78(ode,0:sim.delta_t:sim.t_end,sim.x0,options);
%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
res.q = res.x(:,1:6);
res.p0 = res.x(:,7:12);


% Compute qualtities of interest
res.H = zeros(length(res.t),1);
res.Hsmc = zeros(length(res.t),1);
res.Hp = zeros(length(res.t),1);
res.qd = zeros(length(res.t),6);
res.qe = zeros(length(res.t),6);
res.p =zeros(length(res.t),6);
res.phi=zeros(length(res.t),6);
for i=1:length(res.t)
    res.H(i)    = sys.H(res.q(i,:).',res.p0(i,:).');
    res.p(i,:)  = (ctrl.T(res.q(i,:).')'*res.p0(i,:).').';
    res.Hsmc(i) = ctrl.Hsmc(res.t(i),res.q(i,:).',res.p(i,:).');
    res.qd(i,:) = ctrl.qd(res.t(i)).';
    res.pd(i,:) =ctrl.pd(res.t(i),ctrl.qd(res.t(i))).';
    res.qe(i,:) = res.q(i,:) - res.qd(i,:);
    res.pe(i,:) = ctrl.ep(res.t(i),res.q(i,:).',res.p(i,:).');
    res.phi(i,:)=phi(res.t(i),res.q(i,:).',res.p(i,:).');
    
    res.u(i,:)  =ctrl.u(res.t(i,:).',res.q(i,:).',ctrl.p(res.q(i,:).',res.p0(i,:).'));
     res.match_distur(i,:) =match_distur(res.t(i,:).');
end
disp("运行结束，打印数据")
%%保存数据到指定路径
save('.\Results\Results.mat', 'res');
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
disp("打印数据结束")
% 输出函数
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