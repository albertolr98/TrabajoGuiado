%% Test del entorno
% construcción del entorno
en = entorno;
en = add_pared(en, [0 0], [30 0]);
en = add_pared(en, [30 0], [30 20]);
en = add_pared(en, [30 20], [10 20]);
en = add_pared(en, [10 20], [10 30]);
en = add_pared(en, [10 30], [0 30]);
en = add_pared(en, [0 30], [0 0]);
en = add_pared(en, [0 30], [0 0]);
en = add_pared(en, [0 20], [5 20]);

% construcción del robot
bot = robot([10 10 5*pi/6]);
bot = add_us(bot, [5 2.5 pi/6]);
bot = add_us(bot, [5 -2.5 -pi/6]);

% estimar medidas
[z_estimado, H, X_p] = estimar_medidas(bot, en)

% dibujo

figure;
plot_entorno(en);
plot_robot(bot);

hold on
plot(X_p(:, 1), X_p(:, 2), 'xm', 'MarkerSize', 20);

xlim([-5 35]);
ylim([-5 35]);


