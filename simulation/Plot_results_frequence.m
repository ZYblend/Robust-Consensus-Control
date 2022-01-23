%% Plotting frequency experiment results
LW = 1.3;  % linewidth
FS = 10;   % font size

figure (1)
subplot(5,1,1)
plot(Freq,L2_2x,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent2)')
title('L_2 gain from exogenous input to formation error on X')
set(gca,'FontSize',FS)

subplot(5,1,2)
plot(Freq,L2_3x,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent3)')
set(gca,'FontSize',FS)

subplot(5,1,3)
plot(Freq,L2_4x,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent4)')
set(gca,'FontSize',FS)

subplot(5,1,4)
plot(Freq,L2_5x,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent5)')
set(gca,'FontSize',FS)

subplot(5,1,5)
plot(Freq,L2_6x,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent6)')
set(gca,'FontSize',FS)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure (2)
subplot(5,1,1)
plot(Freq,L2_2y,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent2)')
set(gca,'FontSize',FS)

title('L_2 gain from exogenous input to formation error on Y')
subplot(5,1,2)
plot(Freq,L2_3y,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent3)')
set(gca,'FontSize',FS)

subplot(5,1,3)
plot(Freq,L2_4y,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent4)')
set(gca,'FontSize',FS)

subplot(5,1,4)
plot(Freq,L2_5y,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent5)')
set(gca,'FontSize',FS)

subplot(5,1,5)
plot(Freq,L2_6y,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent6)')
set(gca,'FontSize',FS)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure (3)
subplot(5,1,1)
plot(Freq,L2_2z,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent2)')
set(gca,'FontSize',FS)

title('L_2 gain from exogenous input to formation error on Z')
subplot(5,1,2)
plot(Freq,L2_3z,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent3)')
set(gca,'FontSize',FS)

subplot(5,1,3)
plot(Freq,L2_4z,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent4)')
set(gca,'FontSize',FS)

subplot(5,1,4)
plot(Freq,L2_5z,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent5)')
set(gca,'FontSize',FS)

subplot(5,1,5)
plot(Freq,L2_6z,'ko','LineWidth',LW);
xlabel('Frequency');
ylabel('Gain(agent6)')
set(gca,'FontSize',FS)