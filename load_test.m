clear

load('flight_3.mat');

% model u v w wx wy wz vs
Q = diag([0.100 0.100 0.100 0.001000 0.0010 0.00010 0.0001].^2);
% meas vkx vky vkz va aoa
R = diag([0.5000 0.5000 0.8000 0.5000 .002000 0.002 ].^2);

ki = 0.00000;
alpha = 0.50000;
beta = 2.00000;

dt = 0.1; % s
%start = 2600; % 260s / dt
%stop = 6000; % 600s / dt
time = (t - t(1)) / 1000;

x0 = zeros(1,7);
x0(7) = 1.;
P0 = diag([0.2 0.2 0.2 0.2 0.2 0.2 0.2]);

% sync aoa (if needed)
% aoa_offset = -4.4;
% for i = 2:2:size(AOAangle,1)
%     aoa(i/2) = AOAangle(i)+degtorad(aoa_offset);
% end

% eulers from quat (if needed)
phi = zeros(length(q1), 1);
theta = zeros(length(q1), 1);
psi = zeros(length(q1), 1);
for i = 1:length(q1),
    [psi(i), theta(i), phi(i)] = quat2angle([q1(i) q2(i) q3(i) q4(i)]);
end

% sync imu accel
%axi = [0; IMU_ACCELax(1:length(ax)-1)];
%ayi = [0; IMU_ACCELay(1:length(ay)-1)];
%azi = [0; IMU_ACCELaz(1:length(az)-1)] + 9.81;

offset = start*dt-dt;
%accel = [(time(start:stop)-offset) [axi(start:stop) ayi(start:stop) azi(start:stop)]];
accel = [(time(start:stop)-offset) [ax(start:stop) ay(start:stop) az(start:stop)]];
rates = [(time(start:stop)-offset) [p(start:stop) q(start:stop) r(start:stop)]];
quat = [(time(start:stop)-offset) [q1(start:stop) q2(start:stop) q3(start:stop) q4(start:stop)]];
vk = [(time(start:stop)-offset) [vkx(start:stop) vky(start:stop) vkz(start:stop)]];
va = [(time(start:stop)-offset) va(start:stop)];
aoa = [(time(start:stop)-offset) aoa(start:stop)];
sideslip = [(time(start:stop)-offset) ssa(start:stop)];

% use ground course instead of heading
for i = 1:length(quat)
    gc(i) = atan2(vk(i,3),vk(i,2));
    quat(i,:) = [quat(i,1) angle2quat(gc(i), theta(start-1+i), phi(start-1+i))];
end

% change accel frame (if needed)
% for i = 1:size(accel,1),
%     accel(i,2:4) = rep2(quat(i,2:5)',accel(i,2:4))+[0. 0. 9.81];
% end

%test
w = 100;
for i = start+w:stop-w,
    ss_f(i) = mean(ssa(i-w:i+w));
    aoa_f(i) = mean(aoa(i-w:i+w,2));
end