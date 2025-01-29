classdef BouncingBall < HybridSystem
    % A bouncing ball with an input modeled as a HybridSubsystem subclass.

    % Define variable properties that can be modified.
    properties
        gravity = 9.8;  % Acceleration due to gravity.
        lambda = 0.99; % Coefficient of restitution.

        % Nominal (uncertified) controller gains
        rC = rand;
        rD = rand;

        B; % Control barrier function
        rho; inv_rho; gamma; inv_dgamma; % class-K functions
        ell_gamma; % Legendre-Fenchel Transform

        lambdaD = 0.8; % Krstic \lambda cost disturbance
    end

    % Define constant properties that cannot be modified (i.e., "immutable").
    properties(SetAccess = immutable) 
        % The index of 'height' component
        % within the state vector 'x'.
        height_index = 1;

        % The index of 'velocity' component
        % within the state vector 'x'.
        velocity_index = 2;

        % The index of 'velocity' component
        % within the state vector 'x'.
        cost_index = 3;
    end

    methods 
        %% Constructor
        function obj = BouncingBall(B, rho, inv_rho, gamma, inv_dgamma)
            % Validate CBF object
            obj.B = B;
            validateattributes(B, {'BBControlBarrierFunction'}, {'nonempty'});

            obj.rho = rho;
            obj.inv_rho = inv_rho;
            obj.gamma = gamma;
            obj.inv_dgamma = inv_dgamma;

            % Legendre-Fenchel Transform
            obj.ell_gamma = @(r) r * obj.inv_dgamma(r) - obj.gamma(obj.inv_dgamma(r));

            % Validate function handles
            for func = {rho, inv_rho, gamma, inv_dgamma}
                validateattributes(func{1}, {'function_handle'}, {'nonempty'});
            end
        end

        %% Other methods
        % To define the data of the system, we implement 
        % the abstract functions from HybridSystem.m
        function xdot = flowMap(obj, x, t, j)

            % Extract the state components.
            h = x(obj.height_index);
            v = x(obj.velocity_index);
            
            % Gradient of CBF
            gradB = obj.B.gradB([h; v]);

            % Drift vector
            f = [v; -obj.gravity];
            % Control vector fields in the dynamics (control input)
            fu1 = [0; 1];
            % Disturbance vector fields in the dynamics (disturbance)
            fu2 = [0; 2];
           
            % Nominal feedback law
            bar_kappaC1 = -0.5 * v / obj.rC;

            % ---------------------- Disturbance ---------------------- %
            Lfu2B = gradB' * fu2;
            % Optimal disturbance
            kappaC2 = (obj.lambdaD * obj.inv_dgamma(norm(Lfu2B)) ...
                                                    * Lfu2B / norm(Lfu2B));

            % --------------------- QP Feedback Law --------------------- %
            LfBkappaC1 = dot(gradB, f + fu1*bar_kappaC1);
            Lfu1B = gradB' * fu1;
            omegaC = (LfBkappaC1 + obj.B.alphaC*obj.B.B([h; v]) ...
                                + norm(Lfu2B)*obj.inv_rho(max(0, obj.B.B([h; v]))) ...
                                );

            % if obj.B.B([h; v]) >= obj.rho(obj.B.rho_bar_u)

                % Safeguarding feedback law
                Rc = 0.5 * norm(Lfu1B)^2 / max(0, omegaC);
                hat_kappaC1 = -0.5 * Lfu1B / Rc;
                % Feedback law
                kappaC1 = bar_kappaC1 + hat_kappaC1;

                % ------------------- Cost Evaluation ------------------- %
                q1C = -(LfBkappaC1 - 0.25*norm(Lfu1B)^2/Rc + ...
                                        obj.lambdaD*obj.ell_gamma(norm(Lfu2B)));
                % qC =  (q1C + Rc * norm(hat_kappaC1)^2 ...
                %     - obj.lambdaD * obj.gamma( norm(kappaC2) / obj.lambdaD));
                qC =  (q1C + 0.25*norm(Lfu1B)^2/Rc ...
                    - obj.lambdaD*obj.gamma( norm(kappaC2) / obj.lambdaD));

            % else
            %     kappaC1 = bar_kappaC1;
            %     qC = 0;
            % end

            xdot = [f + fu1*kappaC1 + fu2*kappaC2; qC];
        end


        function xplus = jumpMap(obj, x, t, j)

            % Extract the state components.
            h = x(obj.height_index);
            v = x(obj.velocity_index);
            J = x(obj.cost_index);

            % Drift vector
            g = [h; -obj.lambda * v];
            % System vector fields in the dynamics (control input)
            gu1 = [0; 0.3];
            % System vector fields in the dynamics (disturbance)
            gu2 = [0; 0.08];

            % Nominal feedback law
            bar_kappaD1 = (obj.lambda * v) / (1 + 2 * obj.rD);
            
            % ---------------------- Disturbance ---------------------- %
            BL2 = obj.B.gradB([h; v])' * gu2;
            omegaD = (obj.B.B(g + gu1 * bar_kappaD1) - obj.B.B([h; v]) ...
                + norm(BL2) * obj.inv_rho(obj.B.rho_bar_u) ...
                + obj.B.alphaD*(obj.B.B([h; v]) - obj.B.rho_bar_u));

            % Optimal disturbance
            kappaD2 = obj.lambdaD * obj.inv_dgamma(norm(BL2)) * BL2 / norm(BL2);
 
            % --------------------- QP Feedback Law --------------------- %
            BL1 = obj.B.gradB(x)' * gu1;
            Rd = 0.5 * norm(BL1)^2 / max(0, omegaD);
            hat_kappaD1 = -0.5 * BL1 / Rd;
            % Feedback law
            kappaD1 = bar_kappaD1 + hat_kappaD1;

            % --------------------- Cost Evaluation --------------------- %
            q1D = -(obj.B.B( g + gu1 * bar_kappaD1) - obj.B.B([h; v]) - ...
                0.25*norm(BL1)^2 / Rd + obj.lambdaD*obj.ell_gamma(norm(BL2)) ...
                );
            % qD =  (q1D + Rd * norm(hat_kappaD1)^2 ...
            %         - obj.lambdaD * obj.gamma( norm(kappaD2) / obj.lambdaD));
            qD =  (q1D + 0.25 * norm(BL1)^2 / Rd ...
                    - obj.lambdaD * obj.gamma( norm(kappaD2) / obj.lambdaD));

            % Define the value of G(x).
            xplus = [g + gu1*kappaD1 + gu2*kappaD2; J+qD];
        end

        %%
        function inC = flowSetIndicator(obj, x, t, j)
            % Extract the state components.
            h = x(obj.height_index);
            v = x(obj.velocity_index);

            % Set 'inC' to 1 if 'x' is in the flow set and to 0 otherwise.
            inC = (h >= 0) || (v >= 0);
        end

        function inD = jumpSetIndicator(obj, x, t, j)
            % Extract the state components.
            h = x(obj.height_index);
            v = x(obj.velocity_index);

            % Set 'inD' to 1 if 'x' is in the jump set and to 0 otherwise.
            inD = (h <= 0) && (v <= 0);
        end
    end
end