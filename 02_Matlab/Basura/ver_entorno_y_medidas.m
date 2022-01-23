%% patata
figure(108);
plot_entorno(entorno);
plot_robot(robot);

hold on
plot(X_m(1:length(Z1_k), 1), X_m(1:length(Z1_k), 2), 'xm', 'MarkerSize', 20);

xlim([-2 22]);
ylim([-2 22]);