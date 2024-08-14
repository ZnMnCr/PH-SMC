function [res]=CompulateData(res,sys,ctrl)
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
    res.phi(i,:)=ctrl.phi(res.t(i),res.q(i,:).',res.p(i,:).');
    
    res.u(i,:)  =ctrl.u(res.t(i,:).',res.q(i,:).',ctrl.p(res.q(i,:).',res.p0(i,:).'));
     res.match_distur(i,:) =sys.match_distur(res.t(i,:).');
end