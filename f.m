%-------------------Process_model---------------------
%description :
%
%Dim of the state vector : dim(x) = 6;
%x=[x1 x2 x3 x4 x5 x6]
%x=[uk vk wk uw0 vw0 ww0]
%
%   dx=f(x,u)
%   dx1=f(x1,u)
%   dx2=f(x2,u)
%   ...
%   dx=[dx1; dx2; ...]
%Inputs : 
%		-States:                            x      	
%		-Command inputs :                   omega,a   
%Outputs :
%		-state dot   :                      dx 		
%------------------------------------------------------
function dx = f(x,u,q)
% dot_uvw, dot_wind_bias, dot_airspeed_scale
dx = single(zeros(1,7)');

omega = single(u(1:3));
a = single(rep(q, u(4:6)));
%a = single(u(4:6));

%==> dot_UVW
dx(1) = a(1) + omega(3)* x(2)-omega(2)*x(3);
dx(2) = a(2) + omega(1)* x(3)-omega(3)*x(1);
dx(3) = a(3) + omega(2)* x(1)-omega(1)*x(2);
%  phi = single(atan2(2*(q(1)*q(2)+q(3)*q(4)), 1-2*(q(2)^2+q(3)^2)));
%  theta = single(asin(2*(q(1)*q(3)-q(2)*q(4))));
%  dx(1)=a(1)-9.81*sin(theta)+ omega(3)* x(2)-omega(2)*x(3);
%  dx(2)=a(2)+9.81*cos(theta)*sin(phi) + omega(1)* x(3)-omega(3)*x(1);
%  dx(3)=a(3)+9.81*cos(theta)*cos(phi) + omega(2)* x(1)-omega(1)*x(2);

%==> dot_biais
% already at zero
