function [ctrl]=sakataController(sys,ctrl)
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

ctrl.T =matlabFunction(manualCholesky(inv(sys.M(q_sym))),'vars',{q_sym});

ctrl.p = @(q,p) ctrl.T(q)'*p;
% Compute D, C, G from (5)
ctrl.Dh = @(q) (ctrl.T(q)*sys.D(q))*ctrl.T(q);
ctrl.G = @(q) ctrl.T(q)'*sys.G(q);
ctrl.Ctmp = matlabFunction((jacobian(ctrl.T(q_sym)\p_sym,q_sym).' - jacobian(ctrl.T(q_sym)\p_sym,q_sym)),'vars',[{q_sym}, {p_sym}]);
ctrl.C = @(q,p) (ctrl.T(q)*ctrl.Ctmp(q,p))*ctrl.T(q);
ctrl.D =@(q,p) ctrl.C(q,p)-ctrl.Dh(q);
% Define target trajectory and derivatives
% Compute the target momentum from (13)
%期望动量的坐标变换
ctrl.pd = @(t,q) ctrl.T(q)\ctrl.dqd(t);

% Compute the error coordinates on q, p from (11), (15)
%速度和动量误差
ctrl.ep = @(t,q,p) p - ctrl.pd(t,q);
ctrl.eq = @(t,q) q - ctrl.qd(t);
ctrl.deq = @(t,q,p) ctrl.T(q)'*p - ctrl.dqd(t);
% Compute the gradient of the reference momentum with respect to
% configuration as per (16)
%求期望动量的梯度
ctrl.dpddq = matlabFunction(jacobian(ctrl.pd(t_sym,q_sym),q_sym),'vars',[{t_sym}, {q_sym}]);
%% Define Passivity-based sliding mode controller
%VI. NUMERICAL EXAMPLE Case1 and K
 %K=tril(ones(6));

 K=diag([90000;90000;90000;90000;90000;90000]);
 K2=diag([500;500;500;500;500;300]);

 ctrl.phi=@(t,q,p) (K*ctrl.eq(t,q)+K2*ctrl.ep(t,q,p));%Here q is a variable to be determined
%phi=@(q,p) K*q+tan(p);
%Take a partial derivative of \phi
dphideq=matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),q_sym),'vars',{q_sym});
dphidep=matlabFunction(jacobian(ctrl.phi(t_sym,q_sym,p_sym),p_sym),'vars',{p_sym});
%Replace the variable to be determined q with eq(\tilde \q)

%compute the Lambda from eq.(22)
he=sym(0.5*(dphideq(q_sym)*ctrl.T(q_sym))*dphidep(p_sym)');%这里的系数对系统收敛到滑模面上有影响
%he=100000*0.5*(dphidq(q_sym)*ctrl.T(q_sym))*dphidep(t_sym,q_sym,p_sym)';
Lambda=matlabFunction(2*(he+he'),'vars',[{p_sym}]);
ctrl.Lambda=@(p) Lambda(p);
%
%Compute the partial derivative of U
normPhi=ctrl.phi(t_sym,q_sym,p_sym);
dUdPhi=matlabFunction((ctrl.phi(t_sym,q_sym,p_sym))/(sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);
%Then the feedback controller from eq.(23)
epslion=diag([2.7,2.7,2.7,4.5,4.5,1])*0;
ctrl.v=@(t,q,p) epslion*(ctrl.ep(t,q,p)+dphidep(p)'*dUdPhi(t,q,p))+(-inv(dphidep(p))*Lambda(p)*dUdPhi(t,q,p)+(ctrl.D(q,p))*ctrl.ep(t,q,p)-((inv(dphidep(p))*dphideq(q))*ctrl.T(q))*ctrl.ep(t,q,p));
%ctrl.v=@(t,q,p) (-inv(dphidep(t,q,p))*Lambda(t,q,p)*dUdPhi(t,q,p)+(ctrl.D(q))*ctrl.ep(t,q,p)-((inv(dphidep(t,q,p))*dphidq())*ctrl.T(q))*ctrl.ep(t,q,p));
%ctrl.v=@(t,q,p) zeros(6,1);  
%input u from eq.(20)
ctrl.u = @(t,q,p) ctrl.G(q)\(ctrl.D(q,p)*ctrl.pd(t,q) + ctrl.dpddq(t,q)*(ctrl.T(q)*p) + ctrl.T(q)\ctrl.ddqd(t) + ctrl.T(q)'*sys.dVdq(q) + ctrl.v(t,q,p));

% Define  closed-loop energy in eq.(24)
ctrl.KE = @(t,q,p) 0.5*sum(ctrl.ep(t,q,p).^2);%kinetic energy
ctrl.U=matlabFunction((sqrt(sum(normPhi.^2))),'vars',[{t_sym},{q_sym},{p_sym}]);%potential energy
ctrl.Hd = @(t,q,p) ctrl.KE(t,q,p) + ctrl.U(t,q,p);
