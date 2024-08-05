function [M,P]=massMatrix(m,g)
syms q1 q2 q3 q4 q5 q6 p1 p2 p3 p4 p5 p6 t_sym
q_sym = [q1 q2 q3 q4 q5 q6].';
p_sym = [p1 p2 p3 p4 p5 p6].';

Ixx = 2005;
Iyy = 2005;
Izz = 4000;
Im= [Ixx 0 0;0 Iyy 0;0 0 Izz];
%赋值
%O = @(q) [0.2;0.2;0.2];
O =@(q) [0;0;0];
I = diag([1,1,1]);
R =@(q) [cos(q(6))*cos(q(5))  cos(q(6))*sin(q(5))*sin(q(4))-sin(q(6))*cos(q(4))  cos(q(6))*sin(q(5))*cos(q(4))+sin(q(6))*sin(q(4));
   sin(q(6))*cos(q(5))  sin(q(6))*sin(q(5))*sin(q(4))+cos(q(6))*cos(q(4))  sin(q(6))*sin(q(5))*sin(q(4))-cos(q(6))*sin(q(4));
      -sin(q(5))                         cos(q(5))*sin(q(4))                             cos(q(5))*cos(q(4))        ];
 RO=R(q_sym)*O(q_sym);
inti_RO=[0 -RO(3,:) RO(2,:);RO(3,:) 0 -RO(1,:);-RO(2,:) RO(1,:) 0];  %RO的反对称矩阵
%矩阵
IG = Im-m*inti_RO*inti_RO;
M =matlabFunction([ m*I  -m*inti_RO; m*inti_RO  IG],'vars',[{q_sym}]);
G=[m*g;inti_RO*m*g];
P=matlabFunction((diag(q_sym)*G),'vars',[{q_sym}]);