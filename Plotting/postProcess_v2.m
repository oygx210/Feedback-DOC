function postProcess_v2(sol,color)

%% Figure Formatting
bl_plot_style = {'-.', 'Color', 0.25 * ones(3, 1)};
fs = 15;

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

%% Plot results

time      = sol.phase.time;
state     = sol.phase.state;
control   = sol.phase.control;

figure(1);
plot(time,control*180/pi, bl_plot_style{:});
figure(2);
subplot(1,3,1);
plot(state(:,1),state(:,2), bl_plot_style{:});
subplot(1,3,2);
plot(time,state(:,1), bl_plot_style{:});
subplot(1,3,3);
plot(time,state(:,2), bl_plot_style{:});
figure(3);
subplot(2,2,1);
plot(time,state(:,3), bl_plot_style{:});
subplot(2,2,2);
plot(time,state(:,4), bl_plot_style{:});
subplot(2,2,3);
plot(time,state(:,5), bl_plot_style{:});
subplot(2,2,4);
plot(time,state(:,6), bl_plot_style{:});



end