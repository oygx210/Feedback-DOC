function postProcess_FDOC_v2(sol,color)

%% Figure Formatting
lw   = 1;
fs   =  15;
ms   = 12;

figure(1); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('Steering Angle (deg)','FontSize',fs);
set(gca,'FontSize',fs);

figure(2);
subplot(1,3,1); hold on; grid on;
xlabel('Downstream','FontSize',fs);
ylabel('Upstream','FontSize',fs);
set(gca,'FontSize',fs);
subplot(1,3,2); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('Downstream','FontSize',fs);
set(gca,'FontSize',fs);
subplot(1,3,3); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('Upstream','FontSize',fs);
set(gca,'FontSize',fs);

figure(3);
subplot(2,2,1); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$S_{x_1,p_1}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,2); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$S_{x_2,p_1}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,3); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$S_{x_1,p_2}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,4); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$S_{x_2,p_2}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);

figure(4);
subplot(2,2,1); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$P_{11}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,2); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$P_{21}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,3); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$P_{12}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);
subplot(2,2,4); hold on; grid on;
xlabel('Time','FontSize',fs);
ylabel('$P_{22}$','Interpreter','latex','FontSize',fs);
set(gca,'FontSize',fs);

%% Plot results

time      = sol.phase.time;
state     = sol.phase.state;
control   = sol.phase.control;

figure(1);
plot(time,control*180/pi,'-','color',color,'LineWidth',lw);
figure(2);
subplot(1,3,1);
plot(state(:,1),state(:,2),'-','color',color,'LineWidth',lw);
subplot(1,3,2);
plot(time,state(:,1),'-','color',color,'LineWidth',lw);
subplot(1,3,3);
plot(time,state(:,2),'-','color',color,'LineWidth',lw);
figure(3);
subplot(2,2,1);
plot(time,state(:,3), '-','color',color,'LineWidth',lw);
subplot(2,2,2);
plot(time,state(:,4), '-','color',color,'LineWidth',lw);
subplot(2,2,3);
plot(time,state(:,5), '-','color',color,'LineWidth',lw);
subplot(2,2,4);
plot(time,state(:,6), '-','color',color,'LineWidth',lw);
figure(4);
subplot(2,2,1);
plot(time,state(:,7),'-','color',color,'LineWidth',lw);
subplot(2,2,2);
plot(time,state(:,8),'-','color',color,'LineWidth',lw);
subplot(2,2,3);
plot(time,state(:,9),'-','color',color,'LineWidth',lw);
subplot(2,2,4);
plot(time,state(:,10),'-','color',color,'LineWidth',lw);
    
end