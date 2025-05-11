classdef ValueFunctionApproximator < handle
    % ValueFunctionApproximator: Class for approximating the value function
    % using polynomial basis functions
    
    properties
        basis_order    
        state_dim       
        num_features    
        feature_indices 
    end
    
    methods
        function obj = ValueFunctionApproximator(basis_order, state_dim)
          
            obj.basis_order = basis_order;
            obj.state_dim = state_dim;
        
            obj.feature_indices = obj.generate_feature_indices();
            obj.num_features = size(obj.feature_indices, 1);
        end
        
        function indices = generate_feature_indices(obj)
     
            
         
            indices = zeros(1, obj.state_dim);
         
            for order = 1:obj.basis_order
         
                new_indices = obj.generate_combinations_of_order(order);
                indices = [indices; new_indices];
            end
        end
        
        function combinations = generate_combinations_of_order(obj, order)
        
            
        
            combinations = [];
            
         
            function recurse_combinations(curr_comb, remain_order, start_idx)
                if remain_order == 0
                    combinations = [combinations; curr_comb];
                    return;
                end
                
                for i = start_idx:obj.state_dim
                    new_comb = curr_comb;
                    new_comb(i) = new_comb(i) + 1;
                    recurse_combinations(new_comb, remain_order - 1, i);
                end
            end
    
            recurse_combinations(zeros(1, obj.state_dim), order, 1);
        end
        
        function phi = get_features(obj, x)
     
            x = x(:);
  
            phi = ones(obj.num_features, 1);

            for i = 2:obj.num_features
                term = 1;
                for j = 1:obj.state_dim
                    if obj.feature_indices(i, j) > 0
                        term = term * x(j)^obj.feature_indices(i, j);
                    end
                end
                phi(i) = term;
            end
        end
        
        function value = evaluate(obj, x, W)
            % Evaluate the value function at state x with weights W
            % V(x) = W'Φ(x)
            phi = obj.get_features(x);
            value = W' * phi;
        end
        
        function n = get_num_features(obj)

            n = obj.num_features;
        end
    end
end 