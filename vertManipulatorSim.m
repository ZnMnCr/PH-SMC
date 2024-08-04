%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title: Kinetic-Potential Energy Shaping
% Description: Example implementation of tracking conrtoller from 'Kinetic-potential 
% energy shaping for mechanical systems with applications to tracking',
% submitted to LCSS
% Authours: Joel Ferguson, Alejandro Donaire, Richard H. Middleton
% Version: 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialise
% Clear workspace
clear
close all
clc

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
% Friction model error
sim.frictionMdelError = 0;   % 0 = Perfect friction model; 1 = Imperfect friction model

% Simulation step size
sim.delta_t = 0.01;

% Simulation length
sim.t_end = 20;

% Video output
sim.playVideo = 0;   % 0 = Dont play video; 1 = Play video of result

%% Define 2DOF manipulator model
% symbolic variables
syms q1 q2 p1 p2 t_sym
q_sym = [q1 q2].';
p_sym = [p1 p2].';

% Define kinematic parameters
sys.l1 = 1;
sys.l2 = 1;
sys.G = @(q) [1 -1; 0 1];

% Define inertial parameters
sys.m1 = 3;
sys.m2 = 3;
sys.j1 = 3/12;
sys.j2 = 3/12;
sys.M = @(q) [sys.j1 + (sys.l1^2*sys.m1)/4 + sys.l1^2*sys.m2,   (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2;
                (sys.l1*sys.l2*sys.m2*cos(q(1) - q(2)))/2,                      (sys.m2*sys.l2^2)/4 + sys.j2];
            
% Define friction parameters
sys.dj1 = @(q) 1+1*sin(8*q(1));
sys.dj2 = @(q) 1+1*cos(6*q(1) - 6*q(2));
sys.D = @(q) [sys.dj1(q)+sys.dj2(q), -sys.dj2(q); -sys.dj2(q), sys.dj2(q)];

% Define friction estimates
sys.dj1h = @(q) 1;
sys.dj2h = @(q) 1;
if sim.frictionMdelError
    % Imperfect friction model
    sys.Dh = @(q) [sys.dj1h(q)+sys.dj2h(q), -sys.dj2h(q); -sys.dj2h(q), sys.dj2h(q)];
else
    % Perfect friction model
    sys.Dh = @(q) sys.D(q);
end

% Potential energy parameters
sys.g = 9.8;
sys.V = @(q) sys.g*sys.m2*(sys.l1*sin(q(1)) + (sys.l2*sin(q(2)))/2) + (sys.g*sys.l1*sys.m1*sin(q(1)))/2;

% Define open-loop system energy
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sys.V(q);
sys.T = @(q,p) 0.5*p.'*(sys.M(q)\p);

% Define open-loop energy gradients
sys.dVdq = matlabFunction(jacobian(sys.V(q_sym),q_sym).','vars',{q_sym});
sys.dHdq = matlabFunction(jacobian(sys.H(q_sym,p_sym),q_sym).','vars',[{q_sym}, {p_sym}]);
sys.dHdp = @(q,p) sys.M(q)\p;

% Define system ODE
dx = @(q,p,u) [zeros(2) eye(2); -eye(2) -sys.D(q)]*[sys.dHdq(q,p); sys.dHdp(q,p)] + [zeros(2); sys.G(q)]*u;

%% Control law
% Compute the matrix T^{-1}(q) using the solution from https://en.wikipedia.org/wiki/Square_root_of_a_2_by_2_matrix
ctrl.tau = sys.j1 + (sys.l1^2*sys.m1)/4 + sys.l1^2*sys.m2 + (sys.m2*sys.l2^2)/4 + sys.j2;
ctrl.s = @(q) sqrt(det(sys.M(q)));
ctrl.t = @(q) sqrt(ctrl.tau + 2*ctrl.s(q));
ctrl.Ti = @(q) (sys.M(q)+ctrl.s(q)*eye(2))/ctrl.t(q);

% Compute the transformed momentum in (3)
ctrl.p = @(q,p) ctrl.Ti(q)\p;

% Compute D, C, G from (5)
ctrl.Dh = @(q) (ctrl.Ti(q)\sys.Dh(q))/ctrl.Ti(q);
ctrl.G = @(q) ctrl.Ti(q)\sys.G(q);
ctrl.Ctmp = matlabFunction((jacobian(ctrl.Ti(q_sym)*p_sym,q_sym).' - jacobian(ctrl.Ti(q_sym)*p_sym,q_sym)),'vars',[{q_sym}, {p_sym}]);
ctrl.C = @(q,p) (ctrl.Ti(q)\ctrl.Ctmp(q,p))/ctrl.Ti(q);

% Define target trajectory and derivatives
ctrl.qd = @(t) [pi/2 + (1/2)*cos(t); (1/2)*sin(t)];
ctrl.dqd = @(t) [-(1/2)*sin(t); (1/2)*cos(t)];
ctrl.ddqd = @(t) [-(1/2)*cos(t); -(1/2)*sin(t)];

% Compute the target momentum from (13)
ctrl.pd = @(t,q) ctrl.Ti(q)*ctrl.dqd(t);

% Compute the error coordinates on q, p from (11), (15)
ctrl.ep = @(t,q,p) p - ctrl.pd(t,q);
ctrl.eq = @(t,q) q - ctrl.qd(t);

% Compute the gradient of the reference momentum with respect to
% configuration as per (16)
ctrl.dpddq = matlabFunction(jacobian(ctrl.pd(t_sym,q_sym),q_sym),'vars',[{t_sym}, {q_sym}]);

% Select damping injection terms
ctrl.alpha = 1;
ctrl.Kd = 10*eye(2);

% Define kinetic-potential energy shaping tracking control closed-loop
% energy as per (27)
ctrl.Kp = eye(2);
ctrl.KE = @(t,q,p) 0.5*ctrl.ep(t,q,p).'*ctrl.ep(t,q,p);
ctrl.Vd = @(t,q,p) 0.5*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)).'*ctrl.Kp*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p));
ctrl.Hd = @(t,q,p) ctrl.KE(t,q,p) + ctrl.Vd(t,q,p);

% Define kinetic-potential energy shaping tracking control law as per (17),
% (25)
ctrl.v = @(t,q,p) (ctrl.C(q,p) - ctrl.Dh(q) - ctrl.Kd)*ctrl.alpha*ctrl.Kp*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)) - (ctrl.Ti(q)\ctrl.Kp)*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)) - ctrl.Kd*ctrl.ep(t,q,p);
ctrl.u = @(t,q,p) ctrl.G(q)\(-(ctrl.C(q,p) - ctrl.Dh(q))*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.Ti(q)\p) + ctrl.Ti(q)*ctrl.ddqd(t) + ctrl.Ti(q)\sys.dVdq(q) + ctrl.v(t,q,p));

%% Run simulation
% Define initial conditions
sim.q0 = [0 0].';
sim.p0 = [1 2].';
sim.x0 = [sim.q0; sim.p0];

% Comcatinate model with control law
ode = @(t,x) dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),ctrl.p(x(1:2),x(3:4))));

% Solve ODE
[res.t,res.x] = ode45(ode,[0:sim.delta_t:sim.t_end],sim.x0,odeset('RelTol',1e-6));

%% Plot output
% Unpack solution vector. Solution is in cannonical coordinates
res.q = res.x(:,1:2);
res.p0 = res.x(:,3:4);

% Compute qualtities of interest
res.H = zeros(length(res.t),1);
res.Hd = zeros(length(res.t),1);
res.Hcl = zeros(length(res.t),1);
res.Hp = zeros(length(res.t),1);
res.p0h = zeros(length(res.t),2);
res.qd = zeros(length(res.t),2);
res.qe = zeros(length(res.t),2);
for i=1:length(res.t)
    res.H(i) = sys.H(res.q(i,:).',res.p0(i,:).');
    res.p(i,:) = (ctrl.Ti(res.q(i,:).')\res.p0(i,:).').';
    res.Hd(i) = ctrl.Hd(res.t(i),res.q(i,:).',res.p(i,:).');
    res.qd(i,:) = ctrl.qd(res.t(i)).';
    res.qe(i,:) = res.q(i,:) - res.qd(i,:);
end

% plot energy
fig1 = figure(1)
subplot(2,1,1)
plot(res.t,res.H)
xlabel('time (s)')
ylabel('Open-loop energy')
subplot(2,1,2)
plot(res.t,log(res.Hd))
grid on
xlabel('time (s)')
ylabel('Log of closed-loop energy $H_d$')

% plot configuration
fig2 = figure(2)
plot(res.t,res.q,res.t,res.qd,'--')
legend('$q_1$','$q_2$')
xlabel('time (s)')
ylabel('Configuration')
grid on

if sim.playVideo
    % create video
    res.t1 = res.q(:,1);
    res.x1 = 0.5*sys.l1*cos(res.t1);
    res.y1 = 0.5*sys.l1*sin(res.t1);
    
    res.t2 = res.q(:,2);
    res.x2 = sys.l1*cos(res.t1) + 0.5*sys.l2*cos(res.t2);
    res.y2 = sys.l1*sin(res.t1) + 0.5*sys.l2*sin(res.t2);
    
    res.x_ef = sys.l1*cos(res.t1) + sys.l2*cos(res.t2);
    res.y_ef = sys.l1*sin(res.t1) + sys.l2*sin(res.t2);
    
    fig = figure(3)
    hold on
    % axis equal
    xlim([-2 2]);
    ylim([-2 2]);
    for i=1:length(res.t)
        % clear figure from previous frame
        clf(fig)
        hold on
        xlim([-2 2]);
        ylim([-2 2]);
    
        % plot robot
        plot_beam(res.x1(i),res.y1(i),res.t1(i),sys)
        plot_beam(res.x2(i),res.y2(i),res.t2(i),sys)
    
        % pause to create video
        pause(sim.delta_t)
    end
end

function plot_beam(x,y,t,sys)
    plt.width = 0.1;
    plt.m1Cnr = [0 sys.l1 sys.l1 0 0;
                0 0 plt.width plt.width 0;] - [sys.l1/2; plt.width/2];
    % rotate to correct orientation
    plt.rot = [cos(t) -sin(t); sin(t) cos(t)];
    plt.trans = [x; y];
    plt.m1Cnr = plt.rot*plt.m1Cnr + plt.trans;
    
    plot(plt.m1Cnr(1,:),plt.m1Cnr(2,:))
end