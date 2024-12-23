function dxdt = plant(t,x,sys,ctrl)
    
    
%      [J] = UpdateJacobian(x);
   
%       [T,W]=TensionDistribution(J,u);
    q = x(1:6);
    p = x(7:12);
    q_hat1 = x(13:18);
    p_hat1 = x(19:24);
    distur_hat1 = x(25:30);
    q_hat2 = x(31:36);
    p_hat2 = x(37:42);
    distur_hat2 = x(43:48);
    q_hat3 = x(49:54);
    p_hat3 = x(55:60);
    distur_hat3 = x(61:66);
   kMax = 4;
    k4 = kMax*((x(67) >= kMax) + (x(67) < kMax).* (x(67) / kMax)); 
    if ctrl.selector ==1
     u = ctrl.u(k4,t,q,ctrl.p(q,p));

      else
       u = ctrl.u(t,q,ctrl.p(q,p));
    end

     % CESO State Update
      z_hat1 = [sys.Hdq(q_hat1,p_hat1); sys.Hdp(q_hat1,p_hat1); distur_hat1];     % First Layer ESO State
      z_hat2 = [sys.Hdq(q_hat2,p_hat2); sys.Hdp(q_hat2,p_hat2); distur_hat2];     % Second Layer ESO State
      z_hat3 = [sys.Hdq(q_hat3,p_hat3); sys.Hdp(q_hat3,p_hat3); distur_hat3];     % third Layer ESO State
      z_hat = [z_hat3(1:12);distur_hat1+distur_hat2+distur_hat3];
     % State Space Define
     Ao = [zeros(6), eye(6),   zeros(6);    % ESO State Matrix
           -eye(6), -sys.D(q), zeros(6); 
          zeros(6),   zeros(6), zeros(6)];
    Bo = [zeros(6); sys.G(q); zeros(6)];
    % ESO增益矩阵
    alpha = 5000;        % ESO 的扰动估计增益
    l1 = 3*alpha;           % ESO 增益
    l2 = 3*alpha^2;
    l3 = alpha^3;   
    

     u_comp = u - z_hat(13:end)+sys.match_distur(t);
    
    dxdt =  sys.dx(q,p,u_comp,t);%Define ODE

    Co= [6000*ones(6,1);10000*ones(6,1);9005000*ones(6,1)];
    L1 = [l1*ones(6,1);l2*ones(6,1);l3*ones(6,1)];
    %  z_hat1_p = Ao * z_hat1 + Bo* (u) + Co.* [((abs(q - q_hat1).^0.8).*sign(q - q_hat1)+(q - q_hat1));...
    %  (abs(q - q_hat1).^0.7).*sign(q - q_hat1)+((abs(q - q_hat1).^0.8).*sign(q - q_hat1)+(q - q_hat1));...
    %  (abs(q - q_hat1).^0.5).*sign(q - q_hat1)+(abs(q - q_hat1).^0.6).*sign(q - q_hat1)+(abs(q - q_hat1).^0.7).*sign(q - q_hat1)];

    %  z_hat2_p = Ao * z_hat2 + Bo* (u) + Co.* [((abs(q_hat1 - q_hat2).^0.8).*sign(q_hat1 - q_hat2)+(q_hat1 - q_hat2));...
    %  (abs(q - q_hat1).^0.7).*sign(q_hat1 - q_hat2)+((abs(q_hat1 - q_hat2).^0.7).*sign(q_hat1 - q_hat2)+(q_hat1 - q_hat2));...
    %  (abs(q - q_hat1).^0.7).*sign(q_hat1 - q_hat2)+(abs(q_hat1 - q_hat2).^0.7).*sign(q_hat1 - q_hat2)+(abs(q_hat1 - q_hat2).^0.7).*sign(q_hat1 - q_hat2)];

    %   z_hat3_p = Ao * z_hat2 + Bo* (u) + Co.* [((abs(q_hat2 - q_hat3).^0.8).*sign(q_hat2 - q_hat3)+(q_hat2 - q_hat3));...
    %  (abs(q_hat2 - q_hat3).^0.7).*sign(q_hat2 - q_hat3)+((abs(q_hat2 - q_hat3).^0.7).*sign(q_hat2 - q_hat3)+(q_hat2 - q_hat3));...
    %  (abs(q_hat2 - q_hat3).^0.7).*sign(q_hat2 - q_hat3)+(abs(q_hat2 - q_hat3).^0.7).*sign(q_hat2 - q_hat3)+(abs(q_hat2 - q_hat3).^0.7).*sign(q_hat2 - q_hat3)];
%      z_hat1_p = Ao * z_hat1 + Bo* (u) + L1.*[(q - q_hat1);(q - q_hat1);(q - q_hat1)];
%      z_hat2_p = Ao * z_hat2 + Bo* (u)  + L1.* [(q_hat1 - q_hat2);(q_hat1 - q_hat2);(q_hat1 - q_hat2)];
%      z_hat3_p = Ao * z_hat3 + Bo* (u)  + L1.* [(q_hat2 - q_hat3);(q_hat2 - q_hat3);(q_hat2 - q_hat3)];
     
      m_i = 1;

      
      k4_dot = -m_i*k4+0.0000002*sum(abs(ctrl.phi(t,q,p)));
      z_hat1_p = zeros([18,1]);
      z_hat2_p = zeros([18,1]);
      z_hat3_p = zeros([18,1]);
    dxdt = [dxdt; z_hat1_p; z_hat2_p; z_hat3_p; k4_dot];