%% plotting for results
LW = 1.3;  % linewidth
FS = 10;   % font size

% position
x = out.logsout.getElement('X').Values.Data;
plot3(x(:,1),x(:,2),x(:,3),'LineWidth',LW);
hold on, plot3(x(:,7),x(:,8),x(:,9),'LineWidth',LW);
hold on, plot3(x(:,13),x(:,14),x(:,15),'LineWidth',LW);
hold on, plot3(x(:,19),x(:,20),x(:,21),'LineWidth',LW);
hold on, plot3(x(:,25),x(:,26),x(:,27),'LineWidth',LW);
hold on, plot3(x(:,31),x(:,32),x(:,33),'LineWidth',LW);
legend('agent1','agent2','agent3','agent4','agent5','agent6')
set(gca,'FontSize',FS)

% formation error
t = out.logsout.getElement('e').Values.Time;
e = out.logsout.getElement('e').Values.Data;


x1d = [125 125 0 0 0 0].';
x2d = [250 62.5 62.5*sqrt(3) 0 0 0].';
x3d = [125 0 125*sqrt(3) 0 0 0].';
x4d = [-125 0 125*sqrt(3) 0 0 0].';
x5d = [-250 62.5 62.5*sqrt(3) 0 0 0].';
x6d = [-125 125 0 0 0 0].';



figure
subplot(3,2,1)
plot(t,e(:,1),'LineWidth',LW)
hold on, plot(t,e(:,7),'LineWidth',LW)
hold on, plot(t,e(:,13),'LineWidth',LW)
hold on, plot(t,e(:,19),'LineWidth',LW)
hold on, plot(t,e(:,25),'LineWidth',LW)
hold on, plot(t,e(:,31),'LineWidth',LW)
title('Consensus of loaction \xi_x');
set(gca,'FontSize',FS)

subplot(3,2,3)
plot(t,e(:,2),'LineWidth',LW)
hold on, plot(t,e(:,8),'LineWidth',LW)
hold on, plot(t,e(:,14),'LineWidth',LW)
hold on, plot(t,e(:,20),'LineWidth',LW)
hold on, plot(t,e(:,26),'LineWidth',LW)
hold on, plot(t,e(:,32),'LineWidth',LW)
title('Consensus of loaction \xi_y');
set(gca,'FontSize',FS)

subplot(3,2,5)
plot(t,e(:,3),'LineWidth',LW)
hold on, plot(t,e(:,9),'LineWidth',LW)
hold on, plot(t,e(:,15),'LineWidth',LW)
hold on, plot(t,e(:,21),'LineWidth',LW)
hold on, plot(t,e(:,27),'LineWidth',LW)
hold on, plot(t,e(:,33),'LineWidth',LW)
title('Consensus of loaction \xi_z');
set(gca,'FontSize',FS)

subplot(3,2,2)
plot(t,e(:,4),'LineWidth',LW)
hold on, plot(t,e(:,10),'LineWidth',LW)
hold on, plot(t,e(:,16),'LineWidth',LW)
hold on, plot(t,e(:,22),'LineWidth',LW)
hold on, plot(t,e(:,28),'LineWidth',LW)
hold on, plot(t,e(:,34),'LineWidth',LW)
title('Consensus of velocity \xi_{vx}');
set(gca,'FontSize',FS)

subplot(3,2,4)
plot(t,e(:,5),'LineWidth',LW)
hold on, plot(t,e(:,11),'LineWidth',LW)
hold on, plot(t,e(:,17),'LineWidth',LW)
hold on, plot(t,e(:,23),'LineWidth',LW)
hold on, plot(t,e(:,29),'LineWidth',LW)
hold on, plot(t,e(:,35),'LineWidth',LW)
title('Consensus of velocity \xi_{vy}');
set(gca,'FontSize',FS)

subplot(3,2,6)
plot(t,e(:,6),'LineWidth',LW)
hold on, plot(t,e(:,12),'LineWidth',LW)
hold on, plot(t,e(:,18),'LineWidth',LW)
hold on, plot(t,e(:,24),'LineWidth',LW)
hold on, plot(t,e(:,30),'LineWidth',LW)
hold on, plot(t,e(:,36),'LineWidth',LW)
title('Consensus of velocity \xi_{vz}');
set(gca,'FontSize',FS)



