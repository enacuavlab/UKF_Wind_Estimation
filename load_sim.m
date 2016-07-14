clear

load('sim_13.mat');

% model u v w wx wy wz vs
%Q = diag([0.100 0.100 0.100 0.001000 0.0010 0.00010 0.0001].^2);
Q = diag([0.100 0.100 0.100 0.01000 0.010 0.010 0.0001].^2);
%Q = diag( [1.00000000 1.00000000 1.00000000 0.01000000 0.01000000 0.01000000 0.00000001] );
% meas vkx vky vkz va aoa
%R = diag([0.5000 0.5000 0.8000 0.5000 .002000].^2);
R = diag( [0.25000000 0.25000000 0.25000000 0.25000000 0.000004 ] );

ki = 0.00000;
alpha = 0.50000;
beta = 2.00000;

dt = 0.1; % s
%start = 1110;
%stop = 4400;
time = (t - t(1)) / 1000;

x0 = zeros(1,7);
x0(7) = 1.;
P0 = diag([0.2 0.2 0.2 0.2 0.2 0.2 0.2]);

offset = start*dt-dt;
accel = [(time(start:stop)-offset) [ax(start:stop) ay(start:stop) az(start:stop)]];
rates = [(time(start:stop)-offset) [p(start:stop) q(start:stop) r(start:stop)]];
quat = [(time(start:stop)-offset) [q1(start:stop) q2(start:stop) q3(start:stop) q4(start:stop)]];
vk = [(time(start:stop)-offset) [vkx(start:stop) vky(start:stop) vkz(start:stop)]];
va = [(time(start:stop)-offset) va(start:stop)];
aoa = [(time(start:stop)-offset) aoa(start:stop)];

% change accel frame
accel_b = zeros(stop-start,3);
accel_b2 = zeros(stop-start,3);
for i = 1:size(accel,1),
    accel_b(i,1:3) = rep(quat(i,2:5)',accel(i,2:4)-[0. 0. 9.81]);
    accel_b2(i,1:3) = rep(quat(i,2:5)',accel(i,2:4));
    %test(i,1:3) = accel_b2(i,1:3) - rep(quat(i,2:5)',[0. 0. 9.81]);
    test(i,1:3) = rep(quat(i,2:5)',[0. 0. 9.81]) + accel_b(i,:);
    dx(i,1) = rates(i,4)* v(i+start)-rates(i,3)*w(i+start);
    dx(i,2) = rates(i,2)* w(i+start)-rates(i,4)*u(i+start);
    dx(i,3) = rates(i,3)* u(i+start)-rates(i,2)*v(i+start);
    [psi(i), theta(i), phi(i)] = quat2angle(quat(i,2:5));
    % wind direct
    proj(i,:)=[1 0 tan(aoa(i,2))];
    v_e(i,:) = va(i,2)*rep2(quat(i,2:5)',proj(i,:)');
    wind_d(i,:) = vk(i,2:4)-v_e(i,:);
end

plot(v_e,'DisplayName','v_e')
hold on
plot(vk(:,2:4),'DisplayName','v_k')
plot(wind_d,'DisplayName','wind_d')
plot(va(:,2),'DisplayName','va')
plot(-va(:,2),'DisplayName','va')
figure
plot(radtodeg(psi))
