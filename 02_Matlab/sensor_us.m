classdef sensor_us < sensor
    %SENSOR_US([x_us y_us], beta_us)
    %   Sensor de ultrasonidos.
    
    properties
        angulo_cono
    end
    
    methods
        function obj = sensor_us(X_rel, delta)
            %SENSOR_US([x_rel y_rel theta_rel], delta)
            %   Constructor de un sensor de ultrasonidos, con unas 
            %   posiciones [x_rel y_rel] respecto al centro del robot,  y
            %   apuntando en un ángulo theta_rel. El cono tiene un ángulo
            %   delta de amplitud.
            if isrow(X_rel)
                X_rel = X_rel';
            end
            
            obj.X_rel = X_rel;
            obj.X_rel(3) = wrapToPi(obj.X_rel(3));
            obj.X_abs = zeros(3,1);
            
            obj.angulo_cono = delta;
        end
        
        function obj = actualizar_posicion(obj, X)
            %obj = ACTUALIZAR_POSICION(obj, X)
            %   Actualiza la posición absoluta del sensor, según el vector
            %   X (que es el del robot)
            theta = X(3);
            
            M = [cos(theta) -sin(theta)  0;
                 sin(theta) cos(theta)   0;
                 0          0            1]; % matriz de rotación
            
            obj.X_abs = M*obj.X_rel + X;
            obj.X_abs(3) = wrapToPi(obj.X_abs(3));
        end
        
        
        function plot_us(obj)
            %PLOT_US(obj)
            %   Dibuja el sensor de ultrasonidos, dada la posición del
            %   robot y su ángulo. 
            
            x1 = obj.X_abs(1) + 20*cos(obj.X_abs(3) + obj.angulo_cono);
            y1 = obj.X_abs(2) + 20*sin(obj.X_abs(3) + obj.angulo_cono);
            
            x2 = obj.X_abs(1) + 20*cos(obj.X_abs(3) - obj.angulo_cono);
            y2 = obj.X_abs(2) + 20*sin(obj.X_abs(3) - obj.angulo_cono);
            
            
            hold on
            plot(obj.X_abs(1), obj.X_abs(2), '*r', 'MarkerSize', 10);
            plot([obj.X_abs(1) x1], [obj.X_abs(2) y1], '--r');
            plot([obj.X_abs(1) x2], [obj.X_abs(2) y2], '--r');
            hold off
        end 
        
        %% Estimación de medidas y Jacobiano
        function [z, H, X_m] = estimar_medidas(obj, entorno)
            %[z, H, X_m] = ESTIMAR_MEDIDAS(obj, entorno, robot)
            %   Devuelve la z estimada del sensor (z), así como el
            %   jacobiano de esta (H). Devuelve también las coordenadas del 
            %   punto X_m, para poder ponerlo en una imagen, o donde sea.
            p = entorno.paredes;
            
            x_abs = obj.X_abs(1);
            y_abs = obj.X_abs(2);
            theta_abs = obj.X_abs(3);
            
            x_rel = obj.X_rel(1);
            y_rel = obj.X_rel(2);
            theta_rel = obj.X_rel(3);

            theta = theta_abs - theta_rel;
            dx_dtheta = -x_rel*sin(theta) - y_rel*cos(theta);
            dy_dtheta = x_rel*cos(theta) - y_rel*sin(theta);

            
            delta = obj.angulo_cono;
            
            % medidas
            z = inf; % la distancia usada es la menor de todas las distancias
            H = [0 0 0]; 
            X_m = [0 0];
            
            for i = 1:length(p)
                
                if p(i).tipo == 'h' 
                    %% pared horizontal  
                    % ángulo extremo del cono 1
                    [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs theta_abs+delta], p(i));
                    if new_z < z
                        z = new_z;
                        X_m = new_X_m;
                        
                        % Jacobiano
                        dz_dx = 0;
                        dz_dy = -1/sin(theta_abs + delta);
                        dz_dtheta = 1/sin(theta_abs + delta)^2*(-x_rel*sin(theta_rel + delta) + ...
                            y_rel*cos(theta_rel + delta) + (y_abs - X_m(2))*cos(theta_abs + delta));
                        
                        H = [dz_dx dz_dy dz_dtheta];
                    end
                    
                    % ángulo extremo del cono 2
                    [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs theta_abs-delta], p(i));
                    if new_z < z
                        z = new_z;
                        X_m = new_X_m;
                        
                        % Jacobiano
                        dz_dx = 0;
                        dz_dy = -1/sin(theta_abs - delta);
                        dz_dtheta = 1/sin(theta_abs - delta)^2*(-x_rel*sin(theta_rel - delta) + ...
                            y_rel*cos(theta_rel - delta) + (y_abs - X_m(2))*cos(theta_abs - delta));
                        
                        H = [dz_dx dz_dy dz_dtheta];
                    end
                    
                    % perpendicular 1
                    if abs(wrapToPi(theta_abs - pi/2)) < obj.angulo_cono
                        [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs pi/2], p(i));
                        if new_z < z
                            z = new_z;
                            X_m = new_X_m;
                            
                            % Jacobiano
                            dz_dx = 0;
                            dz_dy = -1;
                            dz_dtheta = -(x_rel*sin(theta_abs) + y_rel*cos(theta_abs));
                            H = [dz_dx dz_dy dz_dtheta];
                        end
                    end
                    
                    % perpendicular 2
                    if abs(wrapToPi(theta_abs + pi/2)) < obj.angulo_cono
                        [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs -pi/2], p(i));
                        if new_z < z
                            z = new_z;
                            X_m = new_X_m;

                            % Jacobiano
                            dz_dx = 0;
                            dz_dy = 1;
                            dz_dtheta = (x_rel*sin(theta_abs) + y_rel*cos(theta_abs));
                            H = [dz_dx dz_dy dz_dtheta];                            
                        end
                    end
                
                elseif p(i).tipo == 'v' 
                    %% pared vertical
                    % ángulo extremo del cono 1
                    [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs theta_abs+delta], p(i));
                    if new_z < z
                        z = new_z;
                        X_m = new_X_m;
                        
                        % Jacobiano
                        dz_dx = -1/cos(theta_abs + delta);
                        dz_dy = 0;
                        dz_dtheta = 1/cos(theta_abs + delta)^2*(-x_rel*sin(theta_rel + delta) + ...
                            y_rel*cos(theta_rel + delta) - (x_abs - X_m(1))*sin(theta_abs + delta));
                        
                        H = [dz_dx dz_dy dz_dtheta];
                    end
                    
                    % ángulo extremo del cono 2
                    [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs theta_abs-delta], p(i));
                    if new_z < z
                        z = new_z;
                        X_m = new_X_m;
                        
                        % Jacobiano
                        dz_dx = -1/cos(theta_abs - delta);
                        dz_dy = 0;
                        dz_dtheta = 1/cos(theta_abs - delta)^2*(-x_rel*sin(theta_rel - delta) + ...
                            y_rel*cos(theta_rel - delta) - (x_abs - X_m(1))*sin(theta_abs - delta));
                        
                        H = [dz_dx dz_dy dz_dtheta];
                    end
                    
                    % perpendicular 1
                    if abs(wrapToPi(theta_abs)) < obj.angulo_cono
                        [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs 0], p(i));
                        if new_z < z
                            z = new_z;
                            X_m = new_X_m;
                            
                            % Jacobiano
                            dz_dx = -1;
                            dz_dy = 0;
                            dz_dtheta = -(x_rel*cos(theta_abs) - y_rel*sin(theta_abs));
                            H = [dz_dx dz_dy dz_dtheta];
                        end
                    end
                    
                    % perpendicular 2
                    if abs(wrapToPi(theta_abs - pi)) < obj.angulo_cono
                        [new_z, new_X_m] = calcular_distancia_haz([x_abs y_abs pi], p(i));
                        if new_z < z
                            z = new_z;
                            X_m = new_X_m;
                            
                            % Jacobiano
                            dz_dx = 1;
                            dz_dy = 0;
                            dz_dtheta = (x_rel*cos(theta_abs) - y_rel*sin(theta_abs));
                            H = [dz_dx dz_dy dz_dtheta];
                        end
                    end
                end
                %% Para ambos tipos de paredes
                % extremo de pared 1
                x_p = p(i).X1(1);
                y_p = p(i).X1(2);
                alpha = atan2(y_p - y_abs, x_p - x_abs); %ángulo con extremo con pared
                if abs(wrapToPi(theta_abs - alpha)) < delta
                    new_z = norm([x_abs-x_p, y_abs - y_p]);
                    if new_z < z
                        z = new_z;
                        X_m = [x_p y_p];

                        % Jacobiano
                        dz_dx = (x_abs - x_p)/z;
                        dz_dy = (y_abs - y_p)/z;
                        dz_dtheta = dz_dx*dx_dtheta + dz_dy*dy_dtheta;
                        H = [dz_dx dz_dy dz_dtheta];

                    end
                end
                    
                % extremo de pared 2
                x_p = p(i).X2(1);
                y_p = p(i).X2(2);
                alpha = atan2(y_p - y_abs, x_p - x_abs); %ángulo con extremo con pared
                if abs(wrapToPi(theta_abs - alpha)) < delta
                    new_z = norm([x_abs-x_p, y_abs - y_p]);
                    if new_z < z
                        z = new_z;
                        X_m = [x_p y_p];
                        
                        % Jacobiano
                        dz_dx = (x_abs - x_p)/z;
                        dz_dy = (y_abs - y_p)/z;
                        dz_dtheta = dz_dx*dx_dtheta + dz_dy*dy_dtheta;
                        H = [dz_dx dz_dy dz_dtheta];

                    end
                end
            end
        end
    end
end

