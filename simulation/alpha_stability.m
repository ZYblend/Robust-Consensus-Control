%% alpha_stable consensus


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

disp(['rank of controllability matrix = ' num2str(rank(ctrb(A,B)))])


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
[U,T] = schur(L);


%% LMI
beta = 1.1;
gamma =0.1;
tau = 0.1;
thre = -(tau*beta^2+1/tau+1/gamma^2+1)/2;


X = sdpvar(n);
Q = sdpvar(m1);

alpha = abs(thre);
H = X*A.'+A*X-2*B*Q*B.'+2*alpha*X; % alpha stability

cons = [H <=0,X>=0, Q>=0];
obj=[];

optimize(cons,obj)

X = value(X);
Q = value(Q);


%% Control design
c = 1/lambda(2)+0.5;
K = -Q*B.'/X;

%% Model uncertainty

Delta2 = eye(n)/2;
Delta3 = zeros(n);
Delta4 =eye(n);
Delta5 = zeros(n);
Delta6 = eye(n);
% Am = kron(eye(N),A);
Am = blkdiag(A,A+Delta2, A+Delta3, A+Delta4, A+Delta5, A+Delta6);
Bm = kron(eye(N),B);
Um = kron(inv(U),eye(n));
% 
disp('check close-loop poles for uncertainty dynamics')
ploe21 = eig(A+Delta2+lambda(2)*c*B*K)
pole31 = eig(A+Delta3+lambda(3)*c*B*K)
ploe41 = eig(A+Delta4+lambda(4)*c*B*K)
pole51 = eig(A+Delta5+lambda(5)*c*B*K)
pole61 = eig(A+Delta6+lambda(6)*c*B*K)
% 
% disp('check close-loop poles for norminal dynamics')
% ploe2 = eig(A+lambda(2)*c*B*K)
% pole3 = eig(A+lambda(3)*c*B*K)
% ploe4 = eig(A+lambda(4)*c*B*K)
% pole5 = eig(A+lambda(5)*c*B*K)
% pole6 = eig(A+lambda(6)*c*B*K)
% 
% disp('compare to the threshold')
% thre = -(tau*beta^2+1/tau+1/beta^2+1)/2


%% for simulation
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
Km = kron(L,c*B*K);

% disturbance
I_tilde = [1 0 0 0 0 0].';
D = kron(I_tilde,ones(n,1));

% 
% run_time=0.1;
% freq = 10;
% Amp=0.1;
