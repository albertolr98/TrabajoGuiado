classdef robot
    %ROBOT([x y], beta)
    %   Robot móvil que va a ser usado. De momento, va a ser solo un
    %   conjunto de sensores unidos a un cuerpo central :). No sé cómo se
    %   podrá simular el movimiento aquí, de momento.
    
    properties
        sensores
        x
        y
        beta
    end
    
    methods
        function obj = robot(P, beta)
            %ROBOT([x y], beta)
            %   Constructor del robot, en una posición [x y], girado un
            %   ángulo beta
            obj.sensores = [];
            obj.x = P(1);
            obj.y = P(2);
            obj.beta = beta;
        end
        
        function obj = add_us(obj, P, beta_us)
            %ADD_US(obj, [x_us y_us], beta_us)
            %   Añade un sensor ultrasonido con unas posiciones [z_us y_us]
            %   respecto al centro del robot, apuntando en un ángulo
            %   beta_us.
            obj.sensores = [obj.sensores sensor_us(P, beta_us)];
        end
        
        function plot_robot(obj)
        	%PLOT_ROBOT(obj) 
            %   Dibuja el robot y sus sensores. 
            hold on
            x2 = obj.x + 100*cos(obj.beta);
            y2 = obj.y + 100*sin(obj.beta);
            plot(obj.x, obj.y, 'hk', 'MarkerSize', 20);
            plot([obj.x x2], [obj.y y2], '-g');
            
            for i = 1:length(obj.sensores)
                plot_us(obj.sensores(i), [obj.x obj.y], obj.beta);
            end
            hold off
        end
        
        function plot_haz(obj, n)
            %PLOT_ROBOT(obj, n) 
            %   Dibuja  el haz de sonido que dispara el sensor n.
            hold on
            plot_haz(obj.sensores(n), [obj.x obj.y], obj.beta);
            hold off
        end
    end
end

