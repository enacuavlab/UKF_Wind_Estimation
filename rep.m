function change = rep(x,C)
%description :
%q-1*C*q 

qinter=zeros(1,4);
qinv=quatinv(x(1:4)');


qinter(1)=-dot(qinv(2:4),C);                                    
qinter(2:4)=cross(qinv(2:4),C) + C*qinv(1); 
change=(cross(qinter(2:4),x(2:4))+ qinter(2:4)*x(1)'+x(2:4)'*qinter(1));

end

