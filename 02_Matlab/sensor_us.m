classdef sensor_us
    %SENSOR_US([x_us y_us], beta_us)
    %   Sensor de ultrasonidos.
    
    properties
        beta_us
        % prefiero trabajar en coordenadas polares por motivos relacionados
        % con el giro del robot, en vez de en cartesianas (x, y).
        r 
        theta
    end
    
    methods
        function obj = sensor_us(P, beta_us)
            %SENSOR_US([x_us y_us], beta_us)
            %   Constructor de un sensor de ultrasonidos, con unas 
            %   posiciones [z_us y_us] respecto al centro del robot,  y
            %   apuntando en un ángulo beta_us.
            obj.beta_us = beta_us;
            [th, r] = cart2pol(P(1), P(2));
            obj.r = r;
            obj.theta = th;
        end
        
        function plot_us(obj, P_r, beta_r)
            %PLOT_US(obj, P_r, beta_r)
            %   Dibuja el sensor de ultrasonidos, dada la posición del
            %   robot y su ángulo. 
            x = P_r(1) + obj.r*cos(obj.theta + beta_r);
            y = P_r(2) + obj.r*sin(obj.theta + beta_r);
            plot(x, y, '*r', 'MarkerSize', 10);
        end
               
        function plot_haz(obj, P_r, beta_r)
            %PLOT_US(obj, P_r, beta_r)
            %   Dibuja  el haz de sonido que dispara el sensor de us.
            hold on
            x = P_r(1) + obj.r*cos(obj.theta + beta_r);
            y = P_r(2) + obj.r*sin(obj.theta + beta_r);
            x2 = x + 500*cos(obj.beta_us + beta_r);
            y2 = y + 500*sin(obj.beta_us + beta_r);
            plot([x x2], [y y2], '--r');
            hold off
        end
    end
end

