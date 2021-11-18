classdef pared_vertical
    %PARED_VERTICAL([x1 y1], [x2 y2])
    %   Pared vertical (x = cte).
    
    properties
        x
        y1
        y2
    end
    
    methods
        function obj = pared_vertical(P1, P2)
            %PARED_VERTICAL([x1 y1], [x2 y2])
            %   Constructor de pared vertical, siendo [x1 y1] las
            %   coordenadas de un extremo; y [x2 y2], las del segundo. x1
            %   debe ser igual que x2 para que sea vertical.
            if P1(1) ~= P2(1)
                error('Las coordenadas x de una pared vertical deben ser iguales');
            end
            obj.x = P1(1);
            obj.y1 = min(P1(2), P2(2));
            obj.y2 = max(P1(2), P2(2));
            
        end
        
        function plot_pared(obj,varargin)
            %PLOT_PARED(obj, ...)
            %   Dibuja la pared.
            plot([obj.x obj.x], [obj.y1 obj.y2], varargin{:});
        end
    end
end

