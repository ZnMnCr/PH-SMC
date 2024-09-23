clear
close all
clc

addpath('./simulation_scripts');


%% Simulation settings
% Simulation step 仿真步长
sim.delta_t = 0.01;
% Simulation length仿真时长
sim.t_end = 5;


syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

%% Define 6DOF CDPR plant
pa.m=6000;%动平台质量
pa.g=[0;0;9.8];%重力加速度
[sys]=DefineCDPR_Plant(pa.m,pa.g);
[ctrl]=DefineTraj();
ctrl.selector = 1;
%% PB-SMC控制算法设计
if ctrl.selector  == 1
    [ctrl]=TSMCController(sys,ctrl);
elseif ctrl.selector  == 2
   [ctrl]=sakataController(sys,ctrl);
elseif ctrl.selector  == 3
    [ctrl]=JoelController(sys,ctrl);
end
%% Run simulation
% Define initial conditions
sim.q0 = [0 0 0 0 0 0].';
sim.p0 = [0 0 0 0 0 0].';
sim.x0 = [sim.q0; sim.p0];

options = odeset('OutputFcn', @myOutputFcn,'RelTol',1e-3);


% Comcatinate model with control law
ode = @(t,x) sys.dx(x(1:6),x(7:12),ctrl.u(t,x(1:6),ctrl.p(x(1:6),x(7:12))),t);
% Solve ODE
tic
[res.t,res.x] = ode78(ode,0:sim.delta_t:sim.t_end,sim.x0,options);
toc
%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
res.q = res.x(:,1:6);
res.p0 = res.x(:,7:12);


% Compute qualtities of interest
[res]=CompulateData(res,sys,ctrl);

disp("运行结束，打印数据")
%保存数据到指定路径
if ctrl.selector == 1
    TSMC = res;
    save('Results/TSMC.mat', 'TSMC');
elseif ctrl.selector == 2
    sakata =res;
    save('Results/sakata.mat', 'sakata');
else
    Joel = res;
    save('Results/Joel.mat', 'Joel');
end

plotAllData(res,ctrl);%出图
disp("打印数据结束")
