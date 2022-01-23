%% formation control

%% agent dynamics
A = [-3 1;
     -5 3];
B = [1 2
     2 1];
 
n = size(A,1);
m1 = size(B,2);

m_out = 2;
C =  eye(2);


% check controllability
disp(['rank of controllability matrix = ' num2str(rank(ctrb(A,B)))])
% check observability
disp(['rank of observability matrix = ' num2str(rank(obsv(A,C)))])

%% Topology
L = [0 0 0 0;
     -1 1 0 0;
     0 -1 1 0;
     0 0 -1 1];

N = size(L,1);

% lambda = eig(L);
% lambda1 = lambda(2);
% lambda2 = lambda(3);
% lambda3 = lambda(4);

%% Static Ouput feedback control
Q = sdpvar(n);
P = sdpvar(n);

B_perp = null(B.');
C_T_perp = null(C);

H1 = B_perp.'*(A*Q+Q*A.')*B_perp;
H2 = C_T_perp.'*(P*A+A.'*P)*C_T_perp;
H3 = [P eye(n);
      eye(n), Q];

slack = -2;
cons = [H1<=slack*eye(size(H1,1)); H2<=slack*eye(size(H2,1)); rank(H3)<=n;P>=0 ];  % LMI constraint
obj = [];

optimize(cons,obj)

Q = value(Q);
P = value(P);

%% Control construction
K_SOFC = -getGain(A,B,C,P,[],1e-6);
K_SOFC = real(K_SOFC);
