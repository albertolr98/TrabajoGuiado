%% Construcción de entorno y robot

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

en = add_baliza(en, [7.8 2.3]);
en = add_baliza(en, [7.8 5.7]);
en = add_baliza(en, [7.8 8.4]);
en = add_baliza(en, [7.8 12.3]);
en = add_baliza(en, [7.8 16.3]);
en = add_baliza(en, [1.4 14.6]);
en = add_baliza(en, [1.4 10]);
en = add_baliza(en, [2 8.6]);
en = add_baliza(en, [2 6.4]);
en = add_baliza(en, [2 4.6]);

save construccion_entorno_robot en

