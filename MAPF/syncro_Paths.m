% FUNCTION synchPaths(~, ~, paths, trans_collisions)
% Purpose: Performs LOCAL temporal synchronization, checking for conflicts 
% ONLY at nodes previously identified as being penalized.
%
% Inputs:
%   paths: Cell array where paths{i} is the node sequence for robot i (spatial only)
%   trans_collisions: Cell array where trans_collisions{i} is a list of 
%                     penalized nodes for robot i.
%
% Output:
%   scheduled_trajectories: Cell array where scheduled_trajectories{i} is a list 
%                           of [node_index, time_step].

function scheduled_trajectories = syncro_Paths(~, ~, paths, trans_collisions)

    num_robots = numel(paths);
    scheduled_trajectories = cell(1, num_robots);
    
    for i = 1:num_robots
        current_path = paths{i};
        if isempty(current_path)
            continue;
        end
        
        % The set of nodes where THIS robot (i) should check for conflicts
        % We only check if the target node (next_node) is in this list.
        % If robot i was not affected, this list is empty.
        local_conflict_nodes = trans_collisions{i}; 
        
        current_schedule = []; 
        current_node = current_path(1);
        time_step = -1; 
        path_index = 1;

        while path_index < numel(current_path) || current_node ~= current_path(end)
            
            % Determine the next planned move
            if current_node == current_path(end)
                 next_node = current_node; 
            else
                 next_node = current_path(path_index + 1);
            end

            wait_needed = false; % Assume no wait is needed initially

            % --- LOCAL CHECK CONDITION ---
            % Only check for conflicts if the NEXT node is one of the penalized nodes
            is_local_contention_point = ismember(next_node, local_conflict_nodes);

            if is_local_contention_point
                
                % Now, check for conflicts, focusing only on the contention point
                
                % We perform a check loop just in case the wait itself causes the conflict to persist
                while true 
                    is_conflict = false;
                    next_time = time_step + 1;
                    
                    % --- CHECK AGAINST ALL HIGHER-PRIORITY ROBOTS (j < i) ---
                    for j = 1:(i-1)
                        high_p_schedule = scheduled_trajectories{j};
                        if isempty(high_p_schedule)
                            continue;
                        end
                        
                        % 1. VERTEX CONFLICT CHECK: Is R_j at next_node at next_time?
                        j_at_next_time = high_p_schedule(high_p_schedule(:, 2) == next_time, 1);
                        
                        if ~isempty(j_at_next_time) && j_at_next_time(1) == next_node
                            is_conflict = true;
                            break; 
                        end
                        
                        % 2. EDGE CONFLICT CHECK: Are R_i and R_j swapping positions?
                        j_at_current_time = high_p_schedule(high_p_schedule(:, 2) == time_step, 1);
                        
                        if ~isempty(j_at_current_time) && ~isempty(j_at_next_time)
                            if j_at_current_time(1) == next_node && j_at_next_time(1) == current_node
                                is_conflict = true;
                                break; 
                            end
                        end
                    end % End j loop
                    
                    if is_conflict
                        % If conflict found, we must WAIT
                        time_step = time_step + 1;
                        current_schedule(end+1, :) = [current_node, time_step];
                        wait_needed = true; 
                        % Continue the while loop to check the *new* next_time
                    else
                        % No conflict found at the local contention point
                        break; 
                    end
                end % End while true (conflict resolution loop)
            end % End if is_local_contention_point
            
            % --- EXECUTE SAFE MOVE (or move after local wait resolution) ---
            time_step = time_step + 1;
            current_node = next_node;
            
            if current_node ~= current_path(end)
                path_index = path_index + 1;
            end
            
            current_schedule(end+1, :) = [current_node, time_step];

        end % End while loop

        % After reaching the goal, wait for visualization
        for w = 1:5 
            time_step = time_step + 1;
            current_schedule(end+1, :) = [current_node, time_step];
        end

        scheduled_trajectories{i} = current_schedule;
    end % End for i = 1:num_robots

end % End function synchPaths