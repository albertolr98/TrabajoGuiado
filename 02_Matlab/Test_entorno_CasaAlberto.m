%% Test del entorno
% construcción del entorno
en = entorno;
en = add_pared(en, [0 0], [0 19.4]);
en = add_pared(en, [0 19.4], [8.0 19.4]);
en = add_pared(en, [8.0 19.4],[8.0 0]);
en = add_pared(en, [8.0 0],[0 0]);

en = add_pared(en, [2.0 0],[2.0 3.2]);
en = add_pared(en, [2.0 4.6],[2.0 5.0]);
en = add_pared(en, [2.0 6.4],[2.0 8.6]);

en = add_pared(en, [2.0 4.6],[8.0 4.6]);
en = add_pared(en, [2.0 6.8],[8.0 6.8]);

en = add_pared(en, [1.4 10.0],[8.0 10.0]);
en = add_pared(en, [1.4 14.6],[8.0 14.6]);

% construcción del robot
bot = robot([5.0 2.5 -pi/6]);
bot = add_us(bot, [0 0 pi/6]);
bot = add_us(bot, [0 0 -pi/6]);

figure;
plot_entorno(en, 'b', 'LineWidth', 3);
plot_robot(bot);
[z_estimado, H] = estimar_medidas(bot, en)

xlim([-5.0 25]);
ylim([-5.0 25]);


