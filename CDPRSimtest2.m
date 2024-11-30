%%
% This script is used to run the simulation of CDPR system with PB-SMC control
% and compare with other control laws.
% Author: HouZongbin
% Email: 1335657223@qq.com

clear
close all
clc

%% Add scripts path
addpath('./simulation_scripts');

%% Simulation settings
% Simulation step 仿真步长
sim.tstart = 0;
sim.delta_t = 0.01;
% Simulation length  仿真时长
sim.t_end = 8;
refine = 4;
kMax = 4.1;
%% Define system and control parameters
% Define symbolic variables
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

% Define 6DOF CDPR plant
pa.m=6000;%Quality of mobile platform
pa.g=[0;0;9.8];%Gravity

[sys]=DefineCDPR_Plant(pa.m,pa.g);% Define plant
[ctrl]=DefineTraj();% Define desired trajectory
%Selector for control law 
ctrl.selector = 1;%1:TSMC;2:PBSMC;3:K-P

%% PB-SMC控制算法设计
if ctrl.selector  == 1
    [ctrl]=TSMCController(sys,ctrl);%TSMC
elseif ctrl.selector  == 2
   [ctrl]=sakataController(sys,ctrl);%PBSMC
elseif ctrl.selector  == 3
    [ctrl]=JoelController(sys,ctrl);%K-PSC
end
%% Run simulation
% Define initial conditions
sim.q0 = [-0 -0 0 0 0 0].';%Define the inital value of configuration
sim.p0 = [0 0 0 0 0 0].';%Define the inital value of momentum
sim.z_hat = zeros(54,1);
sim.adptiveParam = 4;
sim.x0 = [sim.q0; sim.p0; sim.z_hat; sim.adptiveParam];

% options = odeset('OutputFcn', @myOutputFcn,'RelTol',1e-0,'Events', @(t, x) saturationEvent(x, kMax));%Set options for ode solver
options = odeset('OutputFcn', @myOutputFcn,'RelTol',1e-0);%Set options for ode solver

% Comcatinate model with control law
% ode = @(t,x) sys.dx(x(1:6),x(7:12),ctrl.u(t,x(1:6),ctrl.p(x(1:6),x(7:12))),t);%Define ODE function
% Solve ODE
tout = 0;
yout = sim.x0;
% teout = [];
% yeout = [];
% ieout = [];
tic
% while 1
%  [res.t,res.x,te,xe,ie] = ode45(@(t,x) plant(t,x,sys,ctrl),[sim.tstart sim.delta_t sim.t_end ],sim.x0,options);
 [res.t,res.x] = ode45(@(t,x) plant(t,x,sys,ctrl),sim.tstart:sim.delta_t:sim.t_end ,sim.x0,options);
% xe(67)
% nt = length(res.t);

%    tout = [tout; res.t(2:nt)];
%    yout = [yout, res.x(2:nt,:)'];
%    teout = [teout; te];          % Events at tstart are never reported.
%    yeout = [yeout; xe];
%    if sim.tstart >= sim.t_end
%        break;
%    end
%     xe(67) = antiSat(res.x, kMax);
%     sim.x0 = xe;
%     options = odeset(options,'InitialStep',res.t(nt)-res.t(nt-refine),...
%       'MaxStep',res.t(nt)-res.t(1));
   
%    tstart = res.t(nt);

% end
toc
%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
% res.t = tout;
% res.x = yout;
res.q = res.x(:,1:6);
res.p0 = res.x(:,7:12);
res.q_hat1 = res.x(:,13:18);
res.p_hat1 = res.x(:,19:24);
res.dis1 = res.x(:,25:30);
res.q_hat2 = res.x(:,31:36);
res.p_hat2 = res.x(:,37:42);
res.dis2 = res.x(:,43:48);
res.dis3 = res.x(:,61:66);
res.dis =res.dis1+res.dis2+res.dis3;
res.k4 = res.x(:,67);

% Compute qualtities of interest
[res]=CompulateData(res,sys,ctrl); 

disp("运行结束，打印数据")
% Save results
if ctrl.selector == 1
    TSMC = res;
    save('Results/TSMC.mat', 'TSMC');
    disp("TSMC数据保存成功");
elseif ctrl.selector == 2
    sakata =res;
    save('Results/sakata.mat', 'sakata');
    disp("PBSMC数据保存成功");
else
    Joel = res;
    save('Results/Joel.mat', 'Joel');
    disp("K-P数据保存成功");
end
plotAllData(res,ctrl);%Plot results
disp("打印数据结束")
