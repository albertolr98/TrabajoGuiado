classdef entorno
    %ENTORNO Conjunto de paredes
    %   Entorno del robot, definido por una serie de paredes horizontales y
    %   verticales. Para que los algoritmos funcionen, debe ser un entorno
    %   cerrado.
    
    properties
        p_horizontales
        p_verticales
    end
    
    methods
        function obj = entorno
            %ENTORNO 
            %   Constructor del entorno
            obj.p_horizontales = [];
            obj.p_verticales = [];
        end
        
        function obj = add_pared(obj, P1, P2)
            %ADD_PARED([x1 y1], [x2 y2]) 
            %   Añade una pared,  siendo [x1 y1] las coordenadas de un 
            %   extremo; y [x2 y2], las del segundo. La pared debe ser
            %   vertical u horizontal.
            if P1(1) == P2(1) % misma coordenada x
                obj.p_verticales = [obj.p_verticales pared_vertical(P1, P2)];
            elseif P1(2) == P2(2) % misma coordenada y
                obj.p_horizontales = [obj.p_horizontales pared_horizontal(P1, P2)];
            else
                error('La pared debe ser vertical u horizontal');
            end
        end
        
        function plot_entorno(obj, varargin)
            %PLOT_ENTORNO(entorno, ...) 
            %   Dibuja las paredes que constituyen el entorno.
            hold on
            for i = 1:length(obj.p_verticales)
                plot_pared(obj.p_verticales(i), varargin{:});
            end
            for i = 1:length(obj.p_horizontales)
                plot_pared(obj.p_horizontales(i), varargin{:});
            end
            hold off
        end
    end
end

