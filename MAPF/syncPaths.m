function [synched_paths, total_waits] = syncPaths(paths, trans_collisions, colliding_robot_ids)
% SYNCHRONIZEPATHS: Temporal collision detection and robust resolution.
% Implements Prioritized Planning's conflict resolution:
% 1. VERTEX conflict: Insert 1 wait for lower-priority robot (R_k).
% 2. EDGE (Swap) conflict: Insert 2 waits for lower-priority robot (R_k) to solve swap issues.
% The wait is inserted at t_conflict, effectively delaying R_k's move.

fprintf('\n--- Starting Temporal Synchronization and Resolution (V5.2 - Robustness Fix for Failed Paths) ---\n');

% Initialize outputs
synched_paths = paths; 
total_waits = 0; 
N_TOTAL_ROBOTS = numel(paths); 

% ====================================================================
% PHASE 1: INITIAL PADDING & UTILITY SETUP
% ====================================================================

% 1. Determine the initial global maximum path length (makespan).
max_path_length = 0;
for r = 1:N_TOTAL_ROBOTS
    if ~isempty(synched_paths{r})
        max_path_length = max(max_path_length, length(synched_paths{r}));
    end
end

% 2. Pad ALL valid paths up to the initial makespan.
for r = 1:N_TOTAL_ROBOTS
    current_path = synched_paths{r};
    if isempty(current_path)
        % FIX: Skip robots for which no path was found (path is empty).
        continue;
    end
    
    current_length = length(current_path);
    if current_length < max_path_length
        goal_node = current_path(end);
        padding = repmat(goal_node, 1, max_path_length - current_length);
        synched_paths{r} = [current_path, padding];
    end
end

% Initialize Rj_Time_Cache for efficient lookup of high-priority robot times
Rj_Time_Cache = cell(1, N_TOTAL_ROBOTS); 

% --------------------------------------------------------------------
% Local function to insert a wait and update all paths' lengths
% This function correctly inserts 'num_steps' waits (P_k(t_start-1)) at 
% the location t_start, shifting R_k's original action from t_start to t_start + num_steps.
% --------------------------------------------------------------------
function [new_paths, new_max_length] = insert_wait_action(current_paths, robot_k_id, t_start, num_steps, current_max_length)
    
    new_paths = current_paths;
    
    % The node where R_k waits is the node it was at just before the conflict
    % Note: If the path is empty, the main loop's checks should prevent 
    % this function from being called, but we rely on the main check.
    wait_node = new_paths{robot_k_id}(t_start - 1); 
    
    % Insert the wait(s) into R_k's path
    path_k = new_paths{robot_k_id};
    
    % The portion to be inserted is the wait_node repeated 'num_steps' times
    wait_segment = repmat(wait_node, 1, num_steps); 
    
    % Path construction: [start...t_start-1] + [waits] + [t_start...end]
    % This correctly implements the delay at t_start.
    new_paths{robot_k_id} = [path_k(1:t_start-1), wait_segment, path_k(t_start:end)];
    
    % Update global max length
    new_max_length = current_max_length + num_steps; 
    
    % Re-pad ALL other paths to the new max_path_length
    for r = 1:N_TOTAL_ROBOTS
        if r ~= robot_k_id 
            current_path = new_paths{r};
            
            % FIX: Check if the current path is empty before attempting to access (end).
            if isempty(current_path)
                continue; % Skip padding for robots with no path
            end

            % Pad with the last node (goal node)
            goal_node = current_path(end);
            padding = repmat(goal_node, 1, num_steps);
            new_paths{r} = [current_path, padding]; 
        end
    end
end
% --------------------------------------------------------------------


% ====================================================================
% PHASE 2: CONFLICT DETECTION AND RESOLUTION (Priority R_k > R_j)
% ====================================================================

% Iterate over the collision data index (i)
for i = 1:numel(trans_collisions) 
    
    robot_k_idx = colliding_robot_ids(i); % Lower Priority Robot Index (R_k)
    sync_nodes = trans_collisions{i}; 
    
    % --- R_k's time sets (Map: Node -> TimeMap) for O(1) lookup ---
    path_k = synched_paths{robot_k_idx};
    
    % NEW GUARD: If R_k has no path, it cannot be checked for collisions.
    if isempty(path_k)
        fprintf('WARNING: Skipping R%d in synchronization as its path is empty.\n', robot_k_idx);
        continue;
    end

    Rk_SyncTimes = containers.Map('KeyType', 'double', 'ValueType', 'any'); 
    for node_v = sync_nodes
        times_k_array = find(path_k == node_v);
        
        TimeMap_k = containers.Map('KeyType', 'double', 'ValueType', 'logical');
        for t = times_k_array
            TimeMap_k(t) = true;
        end
        Rk_SyncTimes(node_v) = TimeMap_k; 
    end

    wait_inserted = false; 

    % Check R_k against all higher-priority robots R_j (j < k)
    for robot_j_idx = 1 : robot_k_idx - 1 
        
        path_j = synched_paths{robot_j_idx};

        % NEW GUARD: If R_j has no path, it cannot cause a collision with R_k.
        if isempty(path_j)
            continue;
        end
        
        t_conflict = 0; 
        conflict_type = '';
        
        % --- R_j Time Cache Update ---
        if isempty(Rj_Time_Cache{robot_j_idx})
             Rj_Time_Cache{robot_j_idx} = containers.Map('KeyType', 'double', 'ValueType', 'any');
        end
        Rj_Map = Rj_Time_Cache{robot_j_idx};
        
        for node_v = sync_nodes
            if ~Rj_Map.isKey(node_v)
                times_j_array = find(path_j == node_v);
                TimeMap_j = containers.Map('KeyType', 'double', 'ValueType', 'logical');
                for t = times_j_array
                    TimeMap_j(t) = true;
                end
                Rj_Map(node_v) = TimeMap_j; 
            end
        end
        Rj_Time_Cache{robot_j_idx} = Rj_Map; 

        % 1. VERTEX Conflict Check (Same Node, Same Time)
        for node_v = sync_nodes
            if Rk_SyncTimes.isKey(node_v) && Rj_Map.isKey(node_v)
                TimeMap_k = Rk_SyncTimes(node_v); 
                TimeMap_j = Rj_Map(node_v);       
                
                T_j_keys = cell2mat(TimeMap_j.keys); 
                
                for t_j = T_j_keys
                    if TimeMap_k.isKey(t_j) 
                        t_conflict = t_j; 
                        conflict_type = 'VERTEX';
                        
                        fprintf('DETECTED: R%d vs R%d. %s conflict at node %d, time %d.\n', ...
                                robot_k_idx, robot_j_idx, conflict_type, node_v, t_conflict);
                        total_waits = total_waits + 1;
                        break; 
                    end
                end

                if t_conflict ~= 0
                    break; 
                end
            end
        end 
        
        % 2. EDGE Conflict Check (Swap: A->B and B->A, Same Time)
        if t_conflict == 0
            % The check limit is the minimum of the two current path lengths.
            max_t_check = min(length(path_k), length(path_j));
            
            for t = 2:max_t_check 
                node_k_prev = path_k(t-1); 
                node_k_curr = path_k(t);   
                node_j_prev = path_j(t-1); 
                node_j_curr = path_j(t);   
                
                % Check for swap: Rk: A->B and Rj: B->A
                if node_k_prev == node_j_curr && node_k_curr == node_j_prev
                    
                    % Ensure it's a real move and involves a sync node
                    if node_k_prev ~= node_k_curr && any(sync_nodes == node_k_prev) 
                        t_conflict = t;
                        conflict_node_v = node_k_prev; 
                        conflict_type = 'EDGE';
                        
                        fprintf('DETECTED: R%d vs R%d. %s conflict at node %d, time %d (%d->%d vs %d->%d).\n', ...
                                robot_k_idx, robot_j_idx, conflict_type, conflict_node_v, t_conflict, ...
                                node_k_prev, node_k_curr, node_j_prev, node_j_curr);
                        total_waits = total_waits + 1;
                        break; 
                    end
                end
            end
        end
        
        % ========================================================
        % --- RESOLUTION LOGIC: INSERT WAIT(S) FOR R_k ---
        % ========================================================
        % if t_conflict ~= 0
        % 
        %     num_waits = 1; % Default for VERTEX conflict
        %     if strcmp(conflict_type, 'EDGE')
        %         num_waits = 2; % 2 waits for EDGE/Swap to prevent immediate re-collision
        %     end
        % 
        %     % Resolve conflict by inserting wait(s) starting at t_conflict
        %     [synched_paths, max_path_length] = insert_wait_action(synched_paths, robot_k_idx, t_conflict, num_waits, max_path_length);
        %     total_waits = total_waits + num_waits;
        % 
        %     fprintf('RESOLVED: Inserted %d WAIT(s) for R%d starting at time %d. Total waits: %d.\n', ...
        %             num_waits, robot_k_idx, t_conflict, total_waits);
        % 
        %     % Set flag and break the inner R_j loop. Since R_k's path has been modified,
        %     % we must re-evaluate its conflicts from the next entry (i+1).
        %     wait_inserted = true;
        %     break; 
        % end
        
    end % End of R_j loop
end % End of collision data loop (i)

fprintf('--- Temporal Synchronization and Resolution Complete. Total Waits: %d. ---\n', total_waits);
end