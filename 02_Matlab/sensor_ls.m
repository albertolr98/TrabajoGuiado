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

            x_rel = obj.X_rel(1);
            y_rel = obj.X_rel(2);
            theta_rel = obj.X_rel(3);

            theta = theta_abs - theta_rel; % theta del robot

            % medidas
            X_m = [0 0];
            H = [];
            z = [];
            
            for i = 1:length(b)
                % derivadas parciales previas (regla de la cadena) - theta
                % del robot
                dx_dtheta = -x_rel*sin(theta) - y_rel*cos(theta);
                dy_dtheta = x_rel*cos(theta) - y_rel*sin(theta);

                % Ángulo medido phi
                phi = atan2(b(i).X(2) - y_abs, b(i).X(1) - x_abs) - theta_abs;
                dphi_dx = (b(i).X(2)-y_abs)/((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                dphi_dy = -(b(i).X(1)-x_abs)/((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
                dphi_dtheta = dphi_dx*dx_dtheta + dphi_dy*dy_dtheta - 1;
                
                % Distancia
%                 d = sqrt((b(i).X(1) - x_abs)^2 + (b(i).X(2) - y_abs)^2);
%                 dd_dx = (b(i).X(1)-x_abs)/sqrt((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
%                 dd_dy = (b(i).X(2)-y_abs)/sqrt((b(i).X(1)-x_abs)^2+(b(i).X(2)-y_abs)^2);
%                 dd_dtheta = dd_dx*dxdz + dd_dy*dydz;

                % Matriz H (jacobiana)
%                 H = [H;
%                      dphidx dphidy dphidtheta;
%                      dd_dx dd_dy dd_dtheta];
                H = [H;
                     dphi_dx dphi_dy dphi_dtheta];


                % Vector z
%                 z = [z;
%                      phi; 
%                      d];
                z = [z;
                     phi];
            end

            
        end
    end
end