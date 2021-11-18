%% Test del entorno
en = entorno;

en = add_pared(en, [0 0], [300 0]);
en = add_pared(en, [300 0], [300 200]);
en = add_pared(en, [300 200], [100 200]);
en = add_pared(en, [100 200], [100 300]);
en = add_pared(en, [100 300], [0 300]);
en = add_pared(en, [0 300], [0 0]);
en = add_pared(en, [0 300], [0 0]);
en = add_pared(en, [0 200], [50 200]);

bot = robot([150 100], pi*0.8);
bot = add_us(bot, [50 25], pi/6);
bot = add_us(bot, [50 -25], -pi/6);

figure;
plot_entorno(en, 'b', 'LineWidth', 3);
plot_robot(bot);
plot_haz(bot, 1);
plot_haz(bot,2);
xlim([-50 350]);
ylim([-50 350]);


