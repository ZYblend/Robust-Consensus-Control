%% Static Output feeback formation control
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
B = [eye(3);
     eye(3)];
 
n = size(A,1);
m1 = size(B,2);

m_out = 5;
C =  rand(m_out,n);

% check controllability
disp(['rank of controllability matrix = ' num2str(rank(ctrb(A,B)))])
% check observability
disp(['rank of observability matrix = ' num2str(rank(obsv(A,C)))])


%% Topology

L = [0 0 0 0 0 0;
     -1 1 0 0 0 0;
     -1 -1 2 0 0 0;
     -1 0 0 3 -1 -1;
     0 0 0 -1 2 -1;
     -1 0 0 0 0 1;];
     
N = size(L,1);

lambda  = sort(real(eig(L)));
c = 1/lambda(2) + 0.1;

%% LMI
Q = sdpvar(n);
P = sdpvar(n);

B_perp = null(B.');
C_T_perp = null(C);

% alpha =2;
% A = A+alpha*eye(n);

H1 = B_perp.'*(A*Q+Q*A.')*B_perp;
H2 = C_T_perp.'*(P*A+A.'*P)*C_T_perp;
H3 = [P eye(n);
      eye(n), Q];

slack = -5;
cons = [H1<=slack*eye(size(H1,1)); H2<=slack*eye(size(H2,1)); rank(H3)<=n];  % LMI constraint
obj = [];

optimize(cons,obj)

Q = value(Q);
P = value(P);

%% Control construction
K = -getGain(A,B,C,P);

disp('check close-loop poles')
ploe2 = eig(A-lambda(2)*c*B*K*C)
pole3 = eig(A-lambda(3)*c*B*K*C)
ploe4 = eig(A-lambda(4)*c*B*K*C)
pole5 = eig(A-lambda(5)*c*B*K*C)
pole6 = eig(A-lambda(6)*c*B*K*C)

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
Cm = kron(eye(N),C);


% control gain multi-version
Km = kron(L,-c*B*K);









