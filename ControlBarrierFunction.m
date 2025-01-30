classdef ControlBarrierFunction
    % Define ellipse parameters that can be modified.
    properties
        % Semi major and semi minor axis of ellipse
        a; b;
        % alphaC and alphaD defining the class-K functions
        alphaC; alphaD;
        % Upper bound on the disturbance
        rho_bar_u;
    end

    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable)
        % The index of 'height' component
        % within the state vector 'x'.
        height_index = 1;

        % The index of 'velocity' component
        % within the state vector 'x'.
        velocity_index = 2;
    end

    methods
        %% Constructor
        function this = ControlBarrierFunction(a, b, alphaC, alphaD, rho_bar_u)
            % Constructor for instances of the BBControlBarrierFunction class.
            this.a = a;
            this.b = b;
            this.alphaC = alphaC;
            this.alphaD = alphaD;
            this.rho_bar_u = rho_bar_u;

            % Check if alphaC and alphaD are valid
            assert(this.alphaC >= 0, 'alpha_C x must be nonnegative');
            assert(this.alphaD >= 0 && this.alphaD <= 1, ...
                                    'alpha_D x must be between 0 and 1');
            % Check if rho_bar_u is valid
            assert(this.rho_bar_u >= 0, 'rho_bar_u x must be nonnegative');
        end
        
        %% Other methods
        function B = B(this, x)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);
            B = ((h/this.a).^2 + (v/this.b).^2 + ...
                                        h.*v/(this.a * this.b) - 1);
        end

        function gradB = gradB(this, x)
            % Extract the state components.
            h = x(this.height_index);
            v = x(this.velocity_index);

            % Return gradient
            gradB = [2*h/this.a^2 + v/(this.a*this.b); ...
                            2*v/this.b^2 + h/(this.a*this.b)];
        end
        
        %% Helper functions for plotting
        function [xp, yp] = polar_boundary_nbhd(this, which)
            theta = linspace(-pi/2, pi/2, 1000);
            if which == "Kd"
                num = 1 + this.rho_bar_u;
            else
                num = 1;
            end
            r = sqrt( num ./ ...
                            (   (cos(theta)/this.a).^2 + ...
                                (sin(theta)/this.b).^2 + ...
                                sin(theta).*cos(theta)/(this.a*this.b) ...
                            ) ...
                    );
            xp = r .* cos(theta);
            yp = r .* sin(theta);
        end

        function plot(this)
            % Plot the boundary of the 0-superlevel set
            [xK, yK] = this.polar_boundary_nbhd("K");
            % Plot the boundary of the 0-superlevel set
            plot(xK, yK, 'Color', 1/255*[103, 49, 71], 'LineWidth', 2)
            % Plot the 0-sublevel set
            patch(xK, yK, 1/255*[103, 49, 71], 'EdgeColor', 'none', 'FaceAlpha', 0.15);

            
            % Plot the open neighborhood of the boundary of the 0-superlevel set
            [x_nbhd_upper, y_nbhd_upper] = this.polar_boundary_nbhd("Kd");
            [x_nbhd_lower, y_nbhd_lower] = this.polar_boundary_nbhd("K");
            % Fill the area between the upper and lower boundaries
            patch([x_nbhd_lower, flip(x_nbhd_upper)], [y_nbhd_lower, flip(y_nbhd_upper)], 1/255*[218, 165, 32], ...
                                                'EdgeColor', 'none', 'FaceAlpha', 0.15);

            % Plot the boundary of the 0-superlevel set
            plot(x_nbhd_upper, y_nbhd_upper, 'Color', 1/255*[218, 165, 32], 'LineWidth', 2)
        end
    end
end