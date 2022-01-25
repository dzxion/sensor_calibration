function q1 = quat_Integration_StepRK4(q0, omega0, omega1, dt)
% input:
% q0 - k-1ʱ�̵���Ԫ��
% omega0 - k-1ʱ�̵Ľ��ٶ�
% omega1 - kʱ�̵Ľ��ٶ�
% dt - ���ٶȲ���ʱ��
%
% output:
% q1 - kʱ�̵���Ԫ��

omega01 = 0.5 * (omega0 + omega1);
% First Runge-Kutta coefficient
q_omega0 = [0,omega0(1),omega0(2),omega0(3)];
k1 = 0.5 * quaternProd(q0, q_omega0);
% Second Runge-Kutta coefficient
tmp_q = q0 + 0.5 * dt * k1;
q_omega01 = [0,omega01(1),omega01(2),omega01(3)];
k2 = 0.5 * quaternProd(tmp_q, q_omega01);
% Third Runge-Kutta coefficient (same omega skew as second coeff.)
tmp_q = q0 + 0.5 * dt * k2;
k3 = 0.5 * quaternProd(tmp_q, q_omega01);
% Forth Runge-Kutta coefficient
tmp_q = q0 + dt * k3;
q_omega1 = [0,omega1(1),omega1(2),omega1(3)];
k4 = 0.5 * quaternProd(tmp_q, q_omega1);

q1 = q0 + (k1 + 2*k2 + 2*k3 + k4)*dt/6;
q1 = q1 / norm(q1);

end