%% Test del entorno
% construcción del entorno
en = entorno;
en = add_pared(en, [0 0], [0 194]);
en = add_pared(en, [0 194], [80 194]);
en = add_pared(en, [80 194],[80 0]);
en = add_pared(en, [80 0],[0 0]);

en = add_pared(en, [20 0],[20 32]);
en = add_pared(en, [20 46],[20 50]);
en = add_pared(en, [20 64],[20 86]);

en = add_pared(en, [20 46],[80 46]);
en = add_pared(en, [20 68],[80 68]);

en = add_pared(en, [14 100],[80 100]);
en = add_pared(en, [14 146],[80 146]);

% construcción del robot
bot = robot([50 25 -pi/6]);
bot = add_us(bot, [0 0 pi/6]);
bot = add_us(bot, [0 0 -pi/6]);

figure;
plot_entorno(en, 'b', 'LineWidth', 3);
plot_robot(bot);
[z_estimado, H] = estimar_medidas(bot, en)

xlim([-50 350]);
ylim([-50 350]);


