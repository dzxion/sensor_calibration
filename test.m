close all
clear all
clc

% theta = zeros(9,1);
% acc = zeros(3,3);
% L = cost_acc(theta, acc);
% B = zeros(3,3);
% a0 =[0,0,0,0.0048,0.0048,0.0048,0,0,0];
% options=optimset('TolX',1e-6,'TolFun',1e-6,'Algorithm','Levenberg-Marquardt','Display','iter','MaxIter',50);
% % theta = lsqnonlin(@cost_acc,a0,[],[],options,B(:,1:3))
% theta = lsqnonlin(@(theta) cost_acc(theta,B(:,1:3)),a0)

% rng default % for reproducibility
% d = linspace(0,3);
% y = exp(-1.3*d) + 0.05*randn(size(d));
% fun = @(r)exp(-d*r)-y;
% x0 = 4;
% x = lsqnonlin(fun,x0)

for i=1:10
    for i=1:10
        fprintf('helloworld:\n');
    end
end