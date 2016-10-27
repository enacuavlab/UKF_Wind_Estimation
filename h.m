%-----------------Observation_model-------------------

%description :
%
%Dim of the measurement vector dim(zk) = 10;
%y=[y1 y2 y3 y4 y5 y6 y7 y8 y9 y10]
%y=[V1 V2 V3 P1 P2 P3 B1 B2 B3 YH ]
%
%
%Inputs : 
%		-States:                                x      	[15 x 1]
%		-Measures :                             zk		[10  x 1]
%Outputs :
%		-outputs model   :                      y 		[10 x 1]
%------------------------------------------------------
function y = h(x,q)

m = 6;

Vsol = single([0;0;0]);
y = single(zeros(1,m));

%==> Ground speed|R0 = [u,v,w]|R0 + wind|R0
Vsol(1:3) = rep2(q,x(1:3)) + x(4:6)';
%==> Va = scale factor * norm([u,v,w])
Vpitot = x(7) * single(norm(x(1:3)));
%==> AOA = atan(w/u)
alpha = single(atan(x(3)/x(1)));
%==> Sideslip = atan(v/u)
beta = single(asin(x(2)/norm(x)));

y(1:3) = Vsol(1:3);
y(4) = Vpitot;
y(5) = alpha;
y(6) = beta;