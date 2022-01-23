%% Robust consensus against model uncertainty
clear all
clc
%% agent dynamics
omega = 0.15;
A1 = [ 0 0 0;
       0 3*omega^2 0;
       0 0 -omega^2];
A2 = [0 2*omega 0;
      -2*omega 0 0;
      0 0 0];
A = [zeros(3) eye(3);
     A1 A2];
B = [zeros(3);
     eye(3)];
 
n = size(A,1);
m1 = size(B,2);

% m_out = 2;
% C =  eye(2);

% check controllability
disp(['rank of controllability matrix = ' num2str(rank(ctrb(A,B)))])


%% Topology
L = [0 0 0 0 0 0;
     -1 2 0 -1 0 0;
     -1 -1 2 0 0 0;
     -1 0 0 3 -1 -1;
     0 0 0 -1 2 -1;
     -1 0 0 0 0 1];
     
N = size(L,1);

lambda  = sort(real(eig(L)));
c = 1/lambda(2)+1;

[U,T] = schur(L);

%% Model uncertainty
beta = 2.1;
tau = 20;
%% LMI
X = sdpvar(n);
Q = sdpvar(m1);
sdpvar alpha


% H = [A*X+X*A.'-B*Q-Q.'*B.' -eye(n) -eye(n) zeros(n) X;
%      -eye(n) -alpha*eye(n) zeros(n) zeros(n) zeros(n);
%      -eye(n) zeros(n) -tau*eye(n) zeros(n) zeros(n);
%      zeros(n) zeros(n) zeros(n) tau*beta^2*eye(n) zeros(n);
%      X zeros(n) zeros(n) zeros(n) -eye(n)];

% H = [A*X+X*A.'-2*B*Q*B.' eye(n) eye(n) sqrt(1+tau*beta^2)*X;
%      eye(n) -alpha*eye(n) zeros(n) zeros(n);
%      eye(n) zeros(n) -tau*eye(n) zeros(n);
%      sqrt(1+tau*beta^2)*X zeros(n) zeros(n) -eye(n)];

H = [A*X+X*A.'-2*B*Q*B.' eye(n) eye(n) X;
     eye(n) -alpha*eye(n) zeros(n) zeros(n);
     eye(n) zeros(n) -tau*eye(n) zeros(n);
     X zeros(n) zeros(n) -(1/(1+tau*beta^2))*eye(n)];


slack = -2;
cons = [H <=slack*eye(size(H,1)), X>=0, Q>=0,alpha>=0,tau>=0];
obj=[alpha];

% options = sdpsettings('lmirank.solver','sedumi','sedumi.eps',1e-6);
options = sdpsettings('solver','sedumi','sedumi.eps',1e-6);
optimize(cons,obj,options)

X= value(X);
Q = value(Q);
% Q2 = value(Q2);
alpha = value(alpha);

gamma = 2*sqrt(alpha)

%% control design
K = Q*B.'/X;


%% for simulink simulation
% desired formation configuration
x1d = [125 125 0 0 0 0].';
x2d = [250 62.5 62.5*sqrt(3) 0 0 0].';
x3d = [125 0 125*sqrt(3) 0 0 0].';
x4d = [-125 0 125*sqrt(3) 0 0 0].';
x5d = [-250 62.5 62.5*sqrt(3) 0 0 0].';
x6d = [-125 125 0 0 0 0].';
xd = [x1d;
      x2d;
      x3d;
      x4d;
      x5d;
      x6d];
x1dm = kron(ones(N,1),x1d);

% initial states
x1_0 = [0 500 866*sqrt(3) 1.5 0 0].';
x0 = [x1_0;
      zeros(n,1);
      zeros(n,1);
      zeros(n,1);
      zeros(n,1);
      zeros(n,1)];

% multi-agent system dynamics
% Am = kron(eye(N),A);

% Model uncertainty
Delta2 = eye(n)/2;
Delta3 = zeros(n);
Delta4 =eye(n);
Delta5 = zeros(n);
Delta6 = eye(n);
% Am = kron(eye(N),A);
Am = blkdiag(A,A+Delta2, A+Delta3, A+Delta4, A+Delta5, A+Delta6);
% Am = blkdiag(A,A, A+Delta3, A, A, A+Delta6);
Bm = kron(eye(N),B);

disp('check close-loop poles')
ploe2 = eig(A+Delta2-lambda(2)*c*B*K)
pole3 = eig(A+Delta3-lambda(3)*c*B*K)
ploe4 = eig(A+Delta4-lambda(4)*c*B*K)
pole5 = eig(A+Delta5-lambda(5)*c*B*K)
pole6 = eig(A+Delta6-lambda(6)*c*B*K)



% control gain multi-version
Km = kron(L,-c*B*K);

% disturbance
I_tilde = [1 0 0 0 0 0].';
D = kron(I_tilde,ones(n,1));