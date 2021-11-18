classdef pared_horizontal
    %PARED_HORIZONTAL([x1 z1], [y2 y2])
    %   Pared pared_horizontal (y = cte).
    
    properties
        x1
        x2
        y
    end
    
    methods
        function obj = pared_horizontal(P1, P2)
            %PARED_VERTICAL([x1 y1], [x2 y2])
            %   Constructor de pared pared_horizontal, siendo [x1 y1] las
            %   coordenadas de un extremo; y [x2 y2], las del segundo. y1
            %   debe ser igual que y2 para que sea pared_horizontal.
            if P1(2) ~= P2(2)
                error('Las coordenadas y de una pared pared_horizontal deben ser iguales');
            end
            obj.x1 = min(P1(1), P2(1));
            obj.x2 = max(P1(1), P2(1));
            obj.y = P1(2);
            
        end
        
        function plot_pared(obj,varargin)
            %PLOT_PARED(obj, ...) 
            %   Dibuja la pared.
            plot([obj.x1 obj.x2], [obj.y obj.y], varargin{:});
        end
    end
end

