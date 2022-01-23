%% Dynamic output feedback formation control
%
% Yu Zheng, Tallahassee, 2021/04/06

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

m_out = 5;
C =  rand(m_out,n);

% % check controllability
% disp(['rank of controllability matrix = ' num2str(rank(ctrb(A,B)))])
% % check observability
% disp(['rank of observability matrix = ' num2str(rank(obsv(A,C)))])


%% Topology

L = [0 0 0 0 0 0;
     -1 1 0 0 0 0;
     -1 -1 2 0 0 0;
     -1 0 0 3 -1 -1;
     0 0 0 -1 2 -1;
     -1 0 0 0 0 1;];
 
Lf = [1 0 0 0 0;
     -1 2 0 0 0;
     0 0 3 -1 -1;
     0 0 -1 2 -1;
     0 0 0 0 1;];
     
N = size(L,1);

%% augmented system dynamics for dynamic output feedback control
% A0 = kron(eye(N),A);
% Bw = -kron(ones(N,1),eye(n));
% Bw(1:n,:) = zeros(n);
% Bu = kron(eye(N),B);
% Cz = eye(N*n);
% Cy = kron(L,C);
% Dzw = kron(ones(N,1),zeros(n));
% Dzu = zeros(N*n,N*m1);
% Dyw = kron(ones(N,1),zeros(m_out,n));

% G_ol = [A0 Bu Bw;
%         Cz Dzu Dzw];

A0 = kron(eye(N-1),A);
Bw = -kron(ones(N-1,1),eye(n));
Bu = kron(eye(N-1),B);
Cz = eye((N-1)*n);
Cy = kron(Lf,C);
Dzw = kron(ones(N-1,1),zeros(n));
Dzu = zeros((N-1)*n,(N-1)*m1);
Dyw = kron(ones(N-1,1),zeros(m_out,n));

nc=1;
gamma = 2.1;
[K,diagnostics] = DynOutFed_brl2(gamma,A0,Bw,Bu,Cz,Cy,Dzw,Dzu,Dyw,nc);

%% Running bisection
% fun_handle = @(gamma) DynOutFed_brl2(gamma,A0,Bw,Bu,Cz,Cy,Dzw,Dzu,Dyw,nc);
% l = 0;  % lowe bound on gamma
% u = 10;  % upper bound on gamma
% tol = 1e-6; % termination tolerance
% max_iter = 100; % maximum iteration allowed
% 
% [K_bisect,gamma_bisect,flag] = Bisect(fun_handle,l,u,tol,max_iter);