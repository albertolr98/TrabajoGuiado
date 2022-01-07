classdef sensor_ls < sensor
    %SENSOR_LS([x_us y_us], beta_us)
    %   Sensor láser para balizas.
    methods 
        function obj = sensor_ls(X_rel)
            %SENSOR_LS([x_rel y_rel theta_rel])
            %   Constructor de un sensor láser, con unas 
            %   posiciones [x_rel y_rel] respecto al centro del robot,  y
            %   apuntando en un ángulo theta_rel.
            if isrow(X_rel)
                X_rel = X_rel';
            end
            
            obj.X_rel = X_rel;
            obj.X_rel(3) = wrapToPi(obj.X_rel(3));
            obj.X_abs = zeros(3,1);

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
       
        
        %% Estimación de medidas y Jacobiano
        function [z, H, X_m] = estimar_medidas(obj, entorno)
            %[z, H, X_m] = ESTIMAR_MEDIDAS(obj, entorno, robot)
            %   Devuelve la z estimada del sensor (z), así como el
            %   jacobiano de esta (H).
            b = entorno.balizas;
            
            x_abs = obj.X_abs(1);
            y_abs = obj.X_abs(2);
            theta_abs = obj.X_abs(3);

            % medidas
            z = inf; % la distancia usada es la menor de todas las distancias
            X_m = [0 0];
            H = [];
            z = [];
            
            for i = 1:length(b)
                % Theta
                theta = atan2(b(i).X(2) - y_abs, b(i).X(1) - x_abs) - theta_abs;
                tx = (b(i).X(2)-y_abs)/((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                ty = -(b(i).X(1)-x_abs)/((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                tz = -1;
                
                % Distancia
                d = sqrt((b(i).X(1) - x_abs)^2 + (b(i).X(2) - y_abs)^2);
                dx = (b(i).X(1)-x_abs)/sqrt((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                dy = (b(i).X(2)-y_abs)/sqrt((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                dz = 0;

                % Matriz H
                H = [H;
                     tx ty tz;
                     dx dy dz];

                % Vector z
                z = [z;
                     theta; d];
            end
            
        end
    end
end