%% Construcción de entorno y robot
% ángulo del cono del ultrasonido
angulo_cono = 0.174533;

% Contrucción del entorno
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

% Construcción del robot
bot = robot([1 1 0]);
bot = add_us(bot, [0.2 0 0], angulo_cono);
bot = add_us(bot, [0.18 0.11 0.7], angulo_cono);
bot = add_us(bot, [0.18 -0.11 -0.7], angulo_cono);

save construccion_entorno_robot en bot

