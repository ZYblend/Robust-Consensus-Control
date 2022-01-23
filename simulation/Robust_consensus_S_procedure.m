%% Robust consensus against model uncertainty and external disturbance with S procedure
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
c = 1/lambda(2)+0.1;

[U,T] = schur(L);
Um = kron(inv(U),eye(n));

%% Model uncertainty
delta = 0.5;

delta2 = 0.5;
delta3 = 0.1;
delta4 =0.07;
delta5 = 0.03;
delta6 = 0.09;
% Am = kron(eye(N),A);
Am = blkdiag(A,(1+delta2)*A, (1+delta3)*A, (1+delta4)*A, (1+delta5)*A, (1+delta6)*A);
Bm = kron(eye(N),B);

lambdas = 4.6301;
%% LMI
X = sdpvar(n);
Q = sdpvar(m1);
sdpvar alpha
% lambdas = sdpvar;

H1 = [A*X+X*A.'-2*B*Q*B.' eye(n) eye(n);
     eye(n) -lambdas*eye(n) zeros(n)
     eye(n) zeros(n) -alpha*eye(n)];
% H1 = [A*X+X*A.'-lambda(2)*Q.'*B.'-lambda(2)*B*Q eye(n) eye(n);
%      eye(n) -lambdas*eye(n) zeros(n)
%      eye(n) zeros(n) -alpha*eye(n)];
H12 = [X*A.' X;
      zeros(n) zeros(n);
      zeros(n) zeros(n)];
H21 = [A*X zeros(n) zeros(n);
          X zeros(n) zeros(n)];
H3 = [-(1/lambdas)*(1/delta^2)*eye(n) zeros(n);
      zeros(n) -eye(n)];
H = [H1 H12;
     H21 H3];

slack = 0.02;
cons = [H <=-slack*eye(size(H,1)), X>=0, Q>=0, alpha>=0];
obj=[];

options = sdpsettings('solver','sedumi','verbose',1);
% G = optimizer(cons,obj,options,lambdas,{X,Q,alpha});
optimize(cons,obj,options)

%% Line search for lambdas
% lambdas = linspace(0,100,100);
% line_search_result = G(lambdas);
% gamma = sqrt(line_search_result{3});
% plot(lambdas,gamma,'o');


%% for simulink simulation
X= value(X);
Q = value(Q);
% Q2 = value(Q2);
alpha = value(alpha);

gamma = sqrt(alpha)

%% control design
K = Q*B.'/X;
% K = Q/X

disp('check close-loop poles')
ploe2 = eig((1+delta2)*A-lambda(2)*c*B*K)
pole3 = eig((1+delta3)*A-lambda(3)*c*B*K)
ploe4 = eig((1+delta4)*A-lambda(4)*c*B*K)
pole5 = eig((1+delta5)*A-lambda(5)*c*B*K)
pole6 = eig((1+delta6)*A-lambda(6)*c*B*K)

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

% control gain multi-version
Km = kron(L,-c*B*K);

% disturbance
I_tilde = [1 0 0 0 0 0].';
D = kron(I_tilde,ones(n,1));

% run_time=10;
% freq = 1;
% Amp=1;