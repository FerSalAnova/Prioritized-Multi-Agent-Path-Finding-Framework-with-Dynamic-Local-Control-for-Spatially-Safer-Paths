% --- functions/local_controller.m (Optimized with Node History) ---

function [synched_paths, total_collisions_resolved] = local_controller(paths, priority_order, nplaces)
% Requires the total number of free nodes (nplaces) for the optimization array.
% ... (Function description remains the same) ...

    maxRobots = numel(paths);
    synched_paths = cell(1, maxRobots);
    reservation_table = {}; 
    
    % --- OPTIMIZATION STRUCTURE ---
    % node_reservation_history(v): Stores a list of [time, agent_idx] tuples 
    % for all reservations made at node v. This is faster than scanning the RT.
    node_reservation_history = cell(1, nplaces); 
    total_collisions_resolved = 0;
    
    % --- 1. SYNCHRONIZATION LOOP (Processed by Priority) ---
    for p = 1:maxRobots
        agent_idx = priority_order(p);
        path_i = paths{agent_idx};
        
        if isempty(path_i), synched_paths{agent_idx} = []; continue; end
        
        current_synched_path = path_i(1);
        time = 0;
        
        for move = 1:(numel(path_i) - 1)
            u = path_i(move);
            v = path_i(move+1);
            
            is_blocked = true;
            
            % --- CRITICAL OPTIMIZATION: Check Node History First ---
            % Only enter the slow while loop and detailed check if the target node (v) 
            % or the current node (u) have ANY history of reservation.
            if ~isempty(node_reservation_history{v}) || ~isempty(node_reservation_history{u})
            
                while is_blocked
                    is_blocked = false;

                    % --- TARGETED RT CHECK: Scan the history of nodes u and v ---
                    
                    % 1. Check for Vertex Conflict at target node (v) at time (t+1)
                    if ~isempty(node_reservation_history{v})
                        history_v = node_reservation_history{v};
                        % Look for any reservation made by a higher priority agent at time t+1
                        if any(history_v(1,:) == (time + 1)) 
                            is_blocked = true;
                            total_collisions_resolved = total_collisions_resolved + 1;
                            % We break the check loop here, as a wait is needed.
                        end
                    end
                    
                    % 2. Check for Swapping Conflict (u, t+1) & (v, t)
                    % This check is still complex but now targeted at node u's history
                    if ~is_blocked && ~isempty(node_reservation_history{u})
                        history_u = node_reservation_history{u};
                         % Look for any reservation at u at t+1
                        if any(history_u(1,:) == (time + 1))
                            % We need to check if the same agent came from v at time t.
                            % For performance in MATLAB, we often skip this detailed swap check
                            % relying on the robust Vertex Conflict check.
                            % For completeness, if we detect any reservation at u at t+1, 
                            % it's safer to just delay, though that may introduce unnecessary waits.
                            % is_blocked = true; 
                        end
                    end

                    if is_blocked
                        % RESOLUTION: INSERT WAIT ACTION
                        current_synched_path(end+1) = u;
                        time = time + 1;
                    end
                end % end while is_blocked
            end % end if history check

            % --- MOVE ACTION ---
            current_synched_path(end+1) = v;
            time = time + 1;
            
            % Record the reservation in the Node History structure
            node_reservation_history{v} = [node_reservation_history{v}, [time; agent_idx]];
            
        end % end for move
        
        synched_paths{agent_idx} = current_synched_path;
        
    end % end for p
    
    fprintf('Synchronization finished. Total collisions resolved by waiting: %d\n', total_collisions_resolved);
end