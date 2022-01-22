%% patata
load construccion_entorno_robot en bot

en = en.add_pared([6 1.5], [8 1.5])
en = en.add_pared([6 1.5], [6 0])
bot = bot.actualizar_posicion([3.5, 2, 4.5*pi/11-.3])

[z, H, X_m] = estimar_medidas(bot, en)

% X = bot.sensores(6).X_abs;
% x1 = X(1) + 20*cos(X(3) + pi/2 - 0.1);
% y1 = X(2) + 20*sin(X(3) + pi/2 - 0.1);
% x2 = X(1) + 20*cos(X(3) - pi/2 + 0.1);
% y2 = X(2) + 20*sin(X(3) - pi/2 + 0.1);

figure;
plot_robot(bot);
hold on
plot_entorno(en);
hold on
plot(X_m(1:5,1), X_m(1:5,2), '.', 'Color', '#800080', 'MarkerSize', 20)
xlim([-1,8])
ylim([-1,8])

pbaspect([1 1 1])
set(gca,'XColor', 'none','YColor','none')

% set(gca,'visible','off')
% plot(X(1), X(2), 'pb', 'MarkerSize', 20, 'MarkerFaceColor', '#ffa500')
% plot([x1 X(1) x2], [y1 X(2) y2], '--b')

%%
% x0 = 0;
% y0 = 0;
% 
% x0inf = 100;
% y0inf = 0;
% 
% x1 = 10 * cos(pi/7);
% y1 = 10 * sin(pi/7);
% 
% x1inf = 100 * cos(pi/7);
% y1inf = 100 * sin(pi/7);
% 
% x2 = x1 - 5 * sin(pi/7);
% y2 = y1 + 5 * cos(pi/7);
% 
% x2inf = x2 + x1inf;
% y2inf = y2 + y1inf;
% 
% x3inf = x2 + 100 * cos(pi/2.1);
% y3inf = y2 + 100 * sin(pi/2.1);
% 
% figure;
% hold on
% plot(x0, y0, 'hk', 'MarkerSize', 20, 'LineWidth', 1.5)
% plot([x0 x0inf], [y0 y0inf], '--', 'LineWidth', 1.2, 'Color', '#00008B')
% plot([x0 x1 x2], [y0, y1, y2], 'b-', 'LineWidth', 1.5)
% plot(x2, y2, '*r', 'MarkerSize', 20, 'LineWidth', 1.5)
% plot([x1 x1inf], [y1 y1inf], 'k--', 'LineWidth', 1.2)
% plot([x2 x2inf], [y2 y2inf], 'k--', 'LineWidth', 1.2)
% plot([x2 x3inf], [y2 y3inf], '--', 'LineWidth', 1.2, 'Color', '#8B0000')
% xlim([0,20])
% ylim([0,20])
% pbaspect([1 1 1])
% grid on
% set(gca,'XColor', 'none','YColor','none')



