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
bot = robot([20 10 -pi/3]);
bot = add_us(bot, [5 2.5 pi/6]);
bot = add_us(bot, [5 -2.5 -pi/6]);

figure;
plot_entorno(en, 'b', 'LineWidth', 3);
plot_robot(bot);
[z_estimado, H] = estimar_medidas(bot, en)

xlim([-5 35]);
ylim([-5 35]);


