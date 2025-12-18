% --- functions/get_priority_order.m ---

function priority_order = get_priority_order(all_S_Idx, all_G_Idx, initial_adj, heuristic_name)
% Calculates metrics (like path length and static conflicts) for all agents 
% and sorts them based on the heuristic.

    maxRobots = numel(all_S_Idx);
    agent_indices = 1:maxRobots;
    
    % --- 1. Calculate Metrics (Path Lengths and Paths) ---
    path_lengths = zeros(1, maxRobots);
    paths_list = cell(1, maxRobots);
    G_orig = graph(initial_adj);
    
    fprintf('Calculating unconstrained shortest paths for all agents...\n');
    for i = 1:maxRobots
        [path_nodes, path_cost] = shortestpath(G_orig, all_S_Idx(i), all_G_Idx(i));
        path_lengths(i) = path_cost;
        paths_list{i} = path_nodes;
    end
    
    % --- 2. Calculate Static Conflict Count (SCC) ---
    static_conflict_counts = zeros(1, maxRobots);
    
    if contains(lower(heuristic_name), 'conflicting')
        fprintf('Calculating Static Conflict Counts (SCC)...\n');
        
        % Iterate through every pair of agents (i, j)
        for i = 1:maxRobots
            path_i = paths_list{i};
            
            % Skip if no path found (should be handled by feasibility check, but safety first)
            if isempty(path_i)
                static_conflict_counts(i) = Inf; 
                continue;
            end
            
            current_conflicts = 0;
            
            for j = (i + 1):maxRobots
                path_j = paths_list{j};
                
                if isempty(path_j)
                    continue;
                end
                
                % Check for shared nodes (vertex conflicts)
                shared_nodes = intersect(path_i, path_j);
                current_conflicts = current_conflicts + numel(shared_nodes);
                
                % Note: Checking for *edge* conflicts (swaps) is more complex 
                % but often less necessary for this pre-planning heuristic.
            end
            
            % SCC for agent i is the total conflicts found with all other agents
            static_conflict_counts(i) = current_conflicts;
        end
    end
    
    % --- 3. Apply Sorting Heuristic ---
    switch lower(heuristic_name)
        case 'random'
            priority_order = agent_indices(randperm(maxRobots));
            fprintf('Priority set: Random.\n');
            
        case 'shortest_first'
            [~, sorted_idx] = sort(path_lengths, 'ascend');
            priority_order = agent_indices(sorted_idx);
            fprintf('Priority set: Shortest path first (by length).\n');
            
        case 'longest_first'
            [~, sorted_idx] = sort(path_lengths, 'descend');
            priority_order = agent_indices(sorted_idx);
            fprintf('Priority set: Longest path first (by length).\n');
            
        case 'conflicting_first'
            % Sort agents by SCC (descending)
            [~, sorted_idx] = sort(static_conflict_counts, 'descend');
            priority_order = agent_indices(sorted_idx);
            fprintf('Priority set: Most conflicting first (by SCC).\n');
            
        case 'conflicting_last'
            % Sort agents by SCC (ascending)
            [~, sorted_idx] = sort(static_conflict_counts, 'ascend');
            priority_order = agent_indices(sorted_idx);
            fprintf('Priority set: Least conflicting first (by SCC).\n');
            
        case 'original'
             priority_order = agent_indices;
             fprintf('Priority set: Original sequence (1 to %d).\n', maxRobots);

        otherwise
            warning('Unknown heuristic: %s. Using Random priority.', heuristic_name);
            priority_order = agent_indices(randperm(maxRobots));
    end
end