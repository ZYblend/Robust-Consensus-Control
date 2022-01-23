%% Robust Consensus
% clear all
% clc
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
L = [0 0 0 0 0 0;
     -1 2 0 -1 0 0;
     -1 -1 2 0 0 0;
     -1 0 0 3 -1 -1;
     0 0 0 -1 2 -1;
     -1 0 0 0 0 1];
     
N = size(L,1);

lambda  = sort(real(eig(L)));
[U,T] = schur(L);

%% LMI
X = sdpvar(n);
Q = sdpvar(m1);
sdpvar alpha;

% H = [X*A.'+A*X-c*lambda(2)*Q.'*B.'-c*lambda(2)*B*Q eye(n) X;
%      eye(n) -alpha*eye(n) zeros(n);
%      X zeros(n) -eye(n)];
% H = X*A.'+A*X-2*B*Q*B.'+(alpha)*eye(n);
H = [X*A.'+A*X-2*B*Q*B.' -eye(n) X;
     -eye(n) -alpha*eye(n)/4 zeros(n);
     X zeros(n) -eye(n)];

slack = -0.02;
cons = [H <=slack*eye(size(H,1)),X>=0, Q>=0,alpha>=0];
obj=[];

optimize(cons,obj)

X = value(X);
Q = value(Q);
alpha = value(alpha);

gamma = sqrt(4/alpha);

%% Control design
c = 1/lambda(2)+0.1;
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
Am = kron(eye(N),A);
Bm = kron(eye(N),B);

% model uncertainty
% Am = kron(eye(N),A);
delta2 = 0.5;
delta3 = 0.1;
delta4 =0.07;
delta5 = 0.03;
delta6 = 0.09;
% Am = blkdiag(A,(1+delta2)*A, (1+delta3)*A, (1+delta4)*A, (1+delta5)*A, (1+delta6)*A);
Um = kron(U,eye(n));

% control gain multi-version
Km = kron(L,-c*B*K);

% disturbance
I_tilde = [1 0 0 0 0 0].';
D = kron(I_tilde,ones(n,1));

% run_time=4;
% freq = 1;
% Amp=1;
























