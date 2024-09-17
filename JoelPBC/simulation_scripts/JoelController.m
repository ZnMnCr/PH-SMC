function [ctrl]=JoelController(sys)
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

% Define target trajectory and derivatives
%定义轨迹
ctrl.qd = @(t) [-10*(1/2)*cos(t)+5*(1/2)*sin(2*t); 10*(1/2)*cos(t);10*(1/2)*cos(t)+5*(1/2)*sin(3*t);10*(1/2)*cos(t)+5*(1/2)*sin(2*t); -10*(1/2)*cos(t)+5*(1/2)*sin(2*t);10*(1/2)*cos(t)+5*(1/2)*sin(2*t);];
ctrl.dqd = @(t) [10*(1/2)*sin(t)+10*(1/2)*cos(2*t); -10*(1/2)*sin(t);-10*(1/2)*sin(t)+5*0.5*3*cos(3*t);-10*(1/2)*sin(t)+10*(1/2)*cos(2*t);10*(1/2)*sin(t)+10*(1/2)*cos(2*t);-10*(1/2)*sin(t)+10*(1/2)*cos(2*t)];
ctrl.ddqd = @(t) [10*(1/2)*cos(t)-20*(1/2)*sin(2*t); -10*(1/2)*cos(t);-10*(1/2)*cos(t)-5*0.5*3*3*sin(3*t);-10*(1/2)*cos(t)-20*(1/2)*sin(2*t);10*(1/2)*cos(t)-20*(1/2)*sin(2*t);-10*(1/2)*cos(t)-20*(1/2)*sin(2*t)];
% controller child-function
%% Control law
% Compute the matrix T^{-1}(q) using the solution from https://en.wikipedia.org/wiki/Square_root_of_a_2_by_2_matrix
ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});


% Compute the transformed momentum in (3)
ctrl.p = @(q,p) ctrl.T(q)'*p;

% Compute D, C, G from (5)
ctrl.Dh = @(q) (ctrl.T(q)*sys.D(q))*ctrl.T(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);
ctrl.Ctmp = matlabFunction((jacobian(ctrl.T(q_sym)\p_sym,q_sym).' - jacobian(ctrl.T(q_sym)\p_sym,q_sym)),'vars',[{q_sym}, {p_sym}]);
ctrl.C = @(q,p) (ctrl.T(q)*ctrl.Ctmp(q,p))*ctrl.T(q);


% Compute the target momentum from (13)
ctrl.pd = @(t,q) ctrl.T(q)\ctrl.dqd(t);

% Compute the error coordinates on q, p from (11), (15)
ctrl.ep = @(t,q,p) p - ctrl.pd(t,q);
ctrl.eq = @(t,q) q - ctrl.qd(t);

% Compute the gradient of the reference momentum with respect to
% configuration as per (16)
ctrl.dpddq = matlabFunction(jacobian(ctrl.pd(t_sym,q_sym),q_sym),'vars',[{t_sym}, {q_sym}]);

% Select damping injection terms
ctrl.alpha = 0.1;
ctrl.Kd = 20*eye(6);
K= 500*eye(6);
% Define kinetic-potential energy shaping tracking control closed-loop
% energy as per (27)
ctrl.Kp = 100*eye(6);
ctrl.KE = @(t,q,p) 0.5*ctrl.ep(t,q,p).'*ctrl.ep(t,q,p);
ctrl.Vd = @(t,q,p) 0.5*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)).'*ctrl.Kp*(ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p));
ctrl.Hd = @(t,q,p) ctrl.KE(t,q,p) + ctrl.Vd(t,q,p);

% Define kinetic-potential energy shaping tracking control law as per (17),
% (25)
ctrl.v = @(t,q,p) (ctrl.C(q,p) - ctrl.Dh(q) - ctrl.Kd)*ctrl.alpha*ctrl.Kp*(K*ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)) - (ctrl.T(q)*ctrl.Kp)*(K*ctrl.eq(t,q) + ctrl.alpha*ctrl.ep(t,q,p)) - ctrl.Kd*ctrl.ep(t,q,p);
%ctrl.v=@(t,q,p) zeros(6,1);
ctrl.u = @(t,q,p) ctrl.G(q)\(-(ctrl.C(q,p) - ctrl.Dh(q))*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.T(q)*p) + ctrl.T(q)\ctrl.ddqd(t) + ctrl.T(q)'*sys.dVdq(q) + ctrl.v(t,q,p));
