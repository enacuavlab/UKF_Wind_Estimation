function change2=rep2(x,D)
%description :
%q*D*q-1 

qinter=single(zeros(1,4));
qinv=single(quatinv(x(1:4)'));
 
qinter(1) = -dot(x(2:4),D);                                    
qinter(2:4) =cross(x(2:4),D) + D*x(1);                         
change2=single(cross(qinter(2:4),qinv(2:4))+ qinter(2:4)*qinv(1) + qinv(2:4)*qinter(1));

