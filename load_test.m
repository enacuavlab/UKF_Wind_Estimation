clear

load('flight_1.mat');

% model u v w wx wy wz vs
Q = diag([0.100 0.100 0.100 0.001000 0.0010 0.00010 0.0001].^2);
% meas vkx vky vkz va aoa
R = diag([0.5000 0.5000 0.8000 0.5000 .002000].^2);

ki = 0.00000;
alpha = 0.50000;
beta = 2.00000;

dt = 0.1; % s
start = 2600; % 260s / dt
stop = 6000; % 600s / dt
time = (t - t(1)) / 1000;

x0 = zeros(1,7);
x0(7) = 1.;
P0 = diag([0.2 0.2 0.2 0.2 0.2 0.2 0.2]);

% synck aoa
for i = 2:2:size(AOAangle,1)
    aoa(i/2) = AOAangle(i)+degtorad(-4.4);
end

offset = start*dt-dt;
accel = [(time(start:stop)-offset) [ax(start:stop) ay(start:stop) az(start:stop)]];
rates = [(time(start:stop)-offset) [p(start:stop) q(start:stop) r(start:stop)]];
quat = [(time(start:stop)-offset) [q1(start:stop) q2(start:stop) q3(start:stop) q4(start:stop)]];
vk = [(time(start:stop)-offset) [vkx(start:stop) vky(start:stop) vkz(start:stop)]];
va = [(time(start:stop)-offset) va(start:stop)];
aoa = [(time(start:stop)-offset) aoa(start:stop)];

% change accel frame
for i = 1:size(accel,1),
    accel(i,2:4) = rep2(quat(i,2:5)',accel(i,2:4))+[0. 0. 9.81];
end