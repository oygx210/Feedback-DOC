function postProcess_FDOC_mixed(sol,fdoc_plot_style)

%% Figure Formatting
fs   =  15;

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
plot(time,control*180/pi,fdoc_plot_style{:});
figure(2);
subplot(1,3,1);
plot(state(:,1),state(:,2),fdoc_plot_style{:});
subplot(1,3,2);
plot(time,state(:,1),fdoc_plot_style{:});
subplot(1,3,3);
plot(time,state(:,2),fdoc_plot_style{:});
figure(3);
subplot(2,2,1);
plot(time,state(:,3), fdoc_plot_style{:});
subplot(2,2,2);
plot(time,state(:,4), fdoc_plot_style{:});
subplot(2,2,3);
plot(time,state(:,5), fdoc_plot_style{:});
subplot(2,2,4);
plot(time,state(:,6), fdoc_plot_style{:});
figure(4);
subplot(2,2,1);
plot(time,state(:,7),fdoc_plot_style{:});
subplot(2,2,2);
plot(time,state(:,8),fdoc_plot_style{:});
subplot(2,2,3);
plot(time,state(:,9),fdoc_plot_style{:});
subplot(2,2,4);
plot(time,state(:,10),fdoc_plot_style{:});
subplot(2,2,1);
plot(time,state(:,11),fdoc_plot_style{:});
subplot(2,2,2);
plot(time,state(:,12),fdoc_plot_style{:});
subplot(2,2,3);
plot(time,state(:,13),fdoc_plot_style{:});
subplot(2,2,4);
plot(time,state(:,14),fdoc_plot_style{:});
    
end