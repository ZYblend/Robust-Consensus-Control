%% Frequency experiment
clear all
clc

tot = 20;
N = 10;  % running time: 2*pi*N/freq

%% exogenous input
Freq = linspace(10,10e+3,tot);
Amp = 1;


%% system intialization and control design
% alpha_stability
% Robust_consensus_S_procedure
Robust_consensus

%% Run simulink model
L2_2x = zeros(tot,1);
L2_3x = zeros(tot,1);
L2_4x = zeros(tot,1);
L2_5x = zeros(tot,1);
L2_6x = zeros(tot,1);
L2_2y = zeros(tot,1);
L2_3y = zeros(tot,1);
L2_4y = zeros(tot,1);
L2_5y = zeros(tot,1);
L2_6y = zeros(tot,1);
L2_2z = zeros(tot,1);
L2_3z = zeros(tot,1);
L2_4z = zeros(tot,1);
L2_5z = zeros(tot,1);
L2_6z = zeros(tot,1);

for iter=1:tot
    freq = Freq(iter);
    run_time = 1;
    out = sim('Satellite_formation_control');    % run simulink model

    %% calculate L2 gain
    t = out.logsout.getElement('e').Values.Time;
    e = out.logsout.getElement('e').Values.Data;
    d = out.logsout.getElement('d').Values.Data;
    w = out.logsout.getElement('w').Values.Data;

    % formation errors
    e2 = e(:,7:12) - e(:,1:6); 
    e3 = e(:,13:18)- e(:,1:6);
    e4 = e(:,19:24)- e(:,1:6);
    e5 = e(:,25:30)- e(:,1:6);
    e6 = e(:,31:36)- e(:,1:6);
    
    % external disturbance
    w2 = w(:,1:6);
    w3 = w(:,7:12);
    w4 = w(:,13:18);
    w5 = w(:,19:24);
    w6 = w(:,25:30);

    % L2 gain
    % w  ===  e_x
    L2_2x(iter) = norm(e2(:,1))/norm(w2(:,1));
    L2_3x(iter) = norm(e3(:,1))/norm(w3(:,1));
    L2_4x(iter) = norm(e4(:,1))/norm(w4(:,1));
    L2_5x(iter) = norm(e5(:,1))/norm(w5(:,1));
    L2_6x(iter) = norm(e6(:,1))/norm(w6(:,1));
%     L2_2x(iter) = norm(e2(:,1))/norm(d);
%     L2_3x(iter) = norm(e3(:,1))/norm(d);
%     L2_4x(iter) = norm(e4(:,1))/norm(d);
%     L2_5x(iter) = norm(e5(:,1))/norm(d);
%     L2_6x(iter) = norm(e6(:,1))/norm(d);
    
    % w  ===  e_y
    L2_2y(iter) = norm(e2(:,2))/norm(w2(:,2));
    L2_3y(iter) = norm(e3(:,2))/norm(w3(:,2));
    L2_4y(iter) = norm(e4(:,2))/norm(w4(:,2));
    L2_5y(iter) = norm(e5(:,2))/norm(w5(:,2));
    L2_6y(iter) = norm(e6(:,2))/norm(w6(:,2));
%     L2_2y(iter) = norm(e2(:,2))/norm(d);
%     L2_3y(iter) = norm(e3(:,2))/norm(d);
%     L2_4y(iter) = norm(e4(:,2))/norm(d);
%     L2_5y(iter) = norm(e5(:,2))/norm(d);
%     L2_6y(iter) = norm(e6(:,2))/norm(d);
    
    % w  ===  e_z
    L2_2z(iter) = norm(e2(:,3))/norm(w2(:,3));
    L2_3z(iter) = norm(e3(:,3))/norm(w3(:,3));
    L2_4z(iter) = norm(e4(:,3))/norm(w4(:,3));
    L2_5z(iter) = norm(e5(:,3))/norm(w5(:,3));
    L2_6z(iter) = norm(e6(:,3))/norm(w6(:,3));
%     L2_2z(iter) = norm(e2(:,3))/norm(d);
%     L2_3z(iter) = norm(e3(:,3))/norm(d);
%     L2_4z(iter) = norm(e4(:,3))/norm(d);
%     L2_5z(iter) = norm(e5(:,3))/norm(d);
%     L2_6z(iter) = norm(e6(:,3))/norm(d);


end
