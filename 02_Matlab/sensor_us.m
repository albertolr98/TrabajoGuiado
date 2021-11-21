classdef sensor_us
    %SENSOR_US([x_us y_us], beta_us)
    %   Sensor de ultrasonidos.
    
    properties
        X_rel % X relativas al robot
        X_abs % X absolutas
    end
    
    methods
        function obj = sensor_us(X_rel)
            %SENSOR_US([x_rel y_rel theta_rel])
            %   Constructor de un sensor de ultrasonidos, con unas 
            %   posiciones [x_rel y_rel] respecto al centro del robot,  y
            %   apuntando en un ángulo theta_rel.
            if isrow(X_rel)
                X_rel = X_rel';
            end
            
            obj.X_rel = X_rel;
            obj.X_abs = zeros(3,1);
        end
        
        function obj = actualizar_posicion(obj, X)
            %obj = ACTUALIZAR_POSICION(obj, X)
            %   Actualiza la posición absoluta del sensor, según el vector
            %   X (que es el del robot)
            theta = X(3);
            
            M = [cos(theta) -sin(theta)  0;
                 sin(theta) cos(theta)   0;
                 0          0            1];
            
            obj.X_abs = M*obj.X_rel + X;
            obj.X_abs(3) = mod(obj.X_abs(3), 2*pi);
        end
        
        
        function plot_us(obj)
            %PLOT_US(obj)
            %   Dibuja el sensor de ultrasonidos, dada la posición del
            %   robot y su ángulo. 
            hold on
            plot(obj.X_abs(1), obj.X_abs(2), '*r', 'MarkerSize', 10);
            x2 = obj.X_abs(1) + 200*cos(obj.X_abs(3));
            y2 = obj.X_abs(2) + 200*sin(obj.X_abs(3));
            plot([obj.X_abs(1) x2], [obj.X_abs(2) y2], '--r');
            hold off
        end 
        
        %% Estimación de medidas y Jacobiano
        function [z, H] = estimar_medidas(obj, entorno, robot)
            %[z, H] = ESTIMAR_MEDIDAS(obj, entorno, robot)
            %   Devuelve la z estimada del sensor (z), así como el
            %   jacobiano de esta (H).
            p = entorno.paredes;
            x_abs = obj.X_abs(1);
            y_abs = obj.X_abs(2);
            theta_abs = obj.X_abs(3);
            
            % medidas
            z = inf; % la distancia usada es la menor de todas las distancias
            idx = 0; % índice de la pared que se mide
            
            for i = 1:length(p)
                if p(i).tipo == 'h' % pared horizontal
                    y_p = p(i).X1(2);
                    % X corte con la pared
                    x_p = x_abs + 1/tan(theta_abs)*(y_p - y_abs);

                    if ~esta_en_pared(p(i), [x_p; y_p])
                        continue;
                    elseif sign(y_p - y_abs) ~= sign(sin(theta_abs))
                        continue
                    else
                        new_z = (y_p - y_abs)/sin(theta_abs);
                        if new_z < z
                            z = new_z;
                            idx = i;
                        end
                    end
                elseif p(i).tipo == 'v' % pared vertical
                    x_p = p(i).X1(1);
                    % Y corte con la pared
                    y_p = y_abs + tan(theta_abs)*(x_p - x_abs);

                    if ~esta_en_pared(p(i), [x_p; y_p])
                        continue;
                    elseif sign(x_p - x_abs) ~= sign(cos(theta_abs))
                        continue
                    else
                        new_z = (x_p - x_abs)/cos(theta_abs);
                        if new_z < z
                            z = new_z;
                            idx = i;
                        end
                    end
                end
            end
            
            H = jacobiano(obj, p(idx), robot);
        end
        
        function H = jacobiano(obj, pared, robot)
            %H = JACOBIANO(obj, pared, robot)
            %   Calcula la matriz jacobiana de la medida del sensor
            %   respecto a la pared. La pared debe ser la pared respecto a
            %   la que se mide, como es lógico.
            theta_abs = obj.X_abs(3);
            
            x_rel = obj.X_rel(1);
            y_rel = obj.X_rel(2);
            theta_rel = obj.X_rel(3);
            
            if pared.tipo == 'h' % horizontal
                y_p = pared.X1(2);
                y = robot.X(2);
                
                % derivadas parciales dh respecto a...
                dx = 0;
                dy = -1/sin(theta_abs);
                dtheta = 1/sin(theta_abs)^2*(-x_rel*sin(theta_rel) + ...
                    y_rel*cos(theta_rel) + (y - y_p)*cos(theta_abs));
                % la derivada complicada la ha hecho WolframAlpha
            elseif pared.tipo == 'v' % vertical
                x_p = pared.X1(1);
                x = robot.X(1);
                
                % derivadas parciales dh respecto a...
                dx = -1/cos(theta_abs);
                dy = 0;
                dtheta = 1/cos(theta_abs)^2*(-x_rel*sin(theta_rel) + ...
                    y_rel*cos(theta_rel) - (x - x_p)*sin(theta_abs));
                % la derivada complicada la ha hecho WolframAlpha
            end
            
            H = [dx dy dtheta];
        end
        
            
    end
end

