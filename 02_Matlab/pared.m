classdef pared
    %PARED_HORIZONTAL([x1 y1], [y2 y2])
    %   Pared horizontal o vertical (y = cte).
    
    properties
        X1
        X2
        tipo % 'v' o 'h'
    end
    
    methods
        function obj = pared(X1, X2)
            %PARED([x1 y1], [x2 y2])
            %   Constructor de pared,  siendo [x1 y1] las coordenadas de un 
            %   extremo; y [x2 y2], las del segundo. La pared debe ser
            %   vertical (x = cte) u horizontal (y = cte).
            
            if isrow(X1)
                X1 = X1';
            end
            if isrow(2)
                X2 = X2';
            end
            
            if X1(1) == X2(1) % misma coordenada x
                obj.tipo = 'v';
                if X1(2) < X2(2) % hace que siempre X1 tenga el valor más pequeño
                	obj.X1 = X1;
                	obj.X2 = X2; 
                else
                    obj.X1 = X2;
                	obj.X2 = X1; 
                end
            elseif X1(2) == X2(2) % misma coordenada y
                obj.tipo = 'h';
                if X1(1) < X2(1)
                	obj.X1 = X1;
                	obj.X2 = X2; 
                else
                    obj.X1 = X2;
                	obj.X2 = X1; 
                end
            else
                error('La pared debe ser vertical u horizontal');
            end
            
        end
        
        function plot_pared(obj, varargin)
            %PLOT_PARED(obj, ...) 
            %   Dibuja la pared.
            plot([obj.X1(1) obj.X2(1)], [obj.X1(2) obj.X2(2)], varargin{:});
        end
        
        function eep = esta_en_pared(obj, X)
            %eep = ESTA_EN_PARED(obj, X)
            %   Indica si el punto X está en la pared.
            if X(1) < obj.X1(1) || X(1) > obj.X2(1)
                eep = 0;
            elseif X(2) < obj.X1(2) || X(2) > obj.X2(2)
                eep = 0;
            else
                eep = 1;
            end 
        end
    end
end

