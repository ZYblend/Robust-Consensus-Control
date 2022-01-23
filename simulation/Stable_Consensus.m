%% state feedback formation control
clear all
clc
%% agent dynamics
omega = 0.0015;
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
% L = [0 0 0 0 0 0;
%      -7 7 0 0 0 0;
%      -4 -6 10 0 0 0;
%      -5 0 0 18 -7 -6;
%      0 0 0 -7 13 -6;
%      -2 0 0 0 0 2 ];

L = [0 0 0 0 0 0;
     -1 1 0 0 0 0;
     -1 -1 2 0 0 0;
     -1 0 0 3 -1 -1;
     0 0 0 -1 2 -1;
     -1 0 0 0 0 1;];
     
N = size(L,1);

lambda  = sort(real(eig(L)));
c = 1/lambda(2) + 0.1;

% %% LMI
% P = sdpvar(n);
% alpha = 1.1;
% H = A*P+P*A.'-2*B*B.'+2*alpha *P;
% 
% slack = 0;
% cons = [H <=slack*eye(size(H,1)),P>=0];
% obj=[];
% 
% optimize(cons,obj)
% 
% P = value(P);
% K = -B.'*P
% 
% disp('check close-loop poles')
% ploe2 = eig(A+lambda(2)*c*B*K)
% pole3 = eig(A+lambda(3)*c*B*K)
% ploe4 = eig(A+lambda(4)*c*B*K)
% pole5 = eig(A+lambda(5)*c*B*K)
% pole6 = eig(A+lambda(6)*c*B*K)
%% LMI
X = sdpvar(n);
Q = sdpvar(m1);

% H = [X*A.'+A*X-c*lambda(2)*Q.'*B.'-c*lambda(2)*B*Q eye(n) X;
%      eye(n) -alpha*eye(n) zeros(n);
%      X zeros(n) -eye(n)];    % Layapunov

H = X*A.'+A*X-2*B*Q*B.';  % Hurwitz
% alpha = 2;
% H = X*A.'+A*X-2*B*Q*B.'+2*alpha*X; % alpha stability

cons = [H <=0,X>=0, Q>=0];
obj=[];

optimize(cons,obj)

X = value(X);
Q = value(Q);


%% Control design
c = 1/lambda(2)+0.5;
K = Q*B.'/X;

disp('check close-loop poles')
ploe2 = eig(A-lambda(2)*c*B*K)
pole3 = eig(A-lambda(3)*c*B*K)
ploe4 = eig(A-lambda(4)*c*B*K)
pole5 = eig(A-lambda(5)*c*B*K)
pole6 = eig(A-lambda(6)*c*B*K)

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

% initial states
x1_0 = [0 500 866*sqrt(3) 1.5 0 0].';
x0 = [x1_0;
      zeros(n,1);
      zeros(n,1);
      zeros(n,1);
      zeros(n,1);
      zeros(n,1)];

% multi-agent system dynamics
Am = kron(eye(N),A);
Bm = kron(eye(N),B);
% Ab1 = A;
% Ab1(5,2) = 300*omega^2;
% Am = blkdiag(A, Ab1, Ab1, A, A, A);


% control gain multi-version
Km = kron(L,-c*B*K);

% disturbance
I_tilde = [1 0 0 0 0 0].';
D = kron(I_tilde,ones(n,1));






