classdef entorno
    %ENTORNO Conjunto de paredes
    %   Entorno del robot, definido por una serie de paredes horizontales y
    %   verticales. Para que los algoritmos funcionen, debe ser un entorno
    %   cerrado.
    
    properties
        paredes
    end
    
    methods
        function obj = entorno
            %ENTORNO 
            %   Constructor del entorno
            obj.paredes = [];
        end
        
        function obj = add_pared(obj, X1, X2)
            %ADD_PARED([x1 y1], [x2 y2]) 
            %   Añade una pared,  siendo [x1 y1] las coordenadas de un 
            %   extremo; y [x2 y2], las del segundo. La pared debe ser
            %   vertical u horizontal.
            obj.paredes = [obj.paredes pared(X1, X2)];
        end
        
        function plot_entorno(obj, varargin)
            %PLOT_ENTORNO(entorno, ...) 
            %   Dibuja las paredes que constituyen el entorno.
            hold on
            for i = 1:length(obj.paredes)
                plot_pared(obj.paredes(i), varargin{:});
            end
            hold off
        end
    end
end

