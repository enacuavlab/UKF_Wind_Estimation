%----------------Sigma-point---------------------------

%sigmas points et concatenation : n*(2n+1)
%Inputs : 
%		-States:                                 x 		
%		-Covariance on state :                   P 		
%		-Weighting squate-root 
%		-sigmas-point      (Wm,Wc)               c		
%Outputs :  
%		-Sigmas-points (state) :                 X      	
%----------------------------------------------------
function X=sigmas(x,P,c)

%Covariance with ponderation: A = c * P 
A = c*P;

%dimension modification on state 
Y = x(:,ones(1,numel(x)));

%concatenation : X=[x Y+A Y-A]
X = [x Y+A Y-A]; 