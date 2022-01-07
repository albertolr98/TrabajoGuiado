classdef baliza
    %BALIZA(x, y)
    %   Adición de balizas al entorno.
    
    properties
        X
    end
    
    methods
        function obj = baliza(X)
            %BALIZA(x, y)
            %   Constructor de baliza,  siendo x,y sus coordenadas
            if isrow(X)
                X = X';
            end

            obj.X = X;    
        end
        
        function plot_baliza(obj, varargin)
            %PLOT_BALIZA(obj, ...) 
            %   Dibuja la baliza.
            plot(obj.X(1), obj.X(2), varargin{:});
        end
    end
end