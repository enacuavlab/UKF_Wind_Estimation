%----------------Runge Kutta 4-----------------
%description :
%dt is an integration step (tk+1-tk)
%h_2 is define as step/2
%
function xi=RungeKutta(x,dt,u,q)
%Non linear function f(x,u), results : X(:,k)=[Xk+1(0) ... Xk+1(2n+1)]
%=RungeKutta(X(:,k),dt,omega,a);
 
k1 = f(x, u, q);
k2 = f(x+dt*(k1/2), u, q);
k3 = f(x+dt*(k2/2), u, q);
k4 = f(x+dt*k3, u, q);
 
xi = x + (dt/6)*(k1 +2*(k2 + k3) + k4);

