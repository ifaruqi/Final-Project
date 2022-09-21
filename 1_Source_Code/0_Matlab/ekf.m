function [x,P,K]=ekf(fstate,x,u,P,hmeas,z,Q,R)
% EKF   Extended Kalman Filter for nonlinear dynamic systems
% [x, P] = ekf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system:
%           x_k+1 = f(x_k) + w_k
%           z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
% Inputs:   f: function handle for f(x)
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%           h: fanction handle for h(x)
%           z: current measurement
%           Q: process noise covariance 
%           R: measurement noise covariance
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance
%
% By Yi Cao at Cranfield University, 02/01/2008
%

[x1,A]=jaccsd(fstate,x,u);  % Nonlinear update and linearization at current state
P=A*P*A'+Q;                 % Partial update
[z1,H]=jaccsd(hmeas,x1,u);  % Nonlinear measurement and linearization
P12=P*H';                   % Cross covariance
R=chol(H*P12+R);            % Cholesky factorization
U=P12/R;                    % K=U/R'; Faster because of back substitution
K=U/R';                     % Gain
x=x1+U*(R'\(z-z1));         % Back substitution to get state update
P=P-U*U';                   % Covariance update, U*U'=P12/R/R'*P12'=K*P12.

function [z,A]=jaccsd(fun,x,u)
z=fun(x,u);
n=numel(x);
m=numel(z);
A=zeros(m,n);
h=n*eps;
for k=1:n
    x1=x;
    x1(k)=x1(k)+h*1i;
    A(:,k)=imag(fun(x1,u))/h;
end