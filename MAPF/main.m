close all;
clear;
clc;
M = 1000;
FC = 0;
maxRobots = 100;
addpath(['.' filesep 'functions']);

% --- Load map and Scenarios ---
[B, robotPts] = load_map_and_scen('maps/Paris_1_256.map', 'maps/Paris_1_256-random-1.scen');

% --- Select randomly start/goal ---
nTotal = numel(robotPts);
if maxRobots > nTotal
    warning('maxRobots (%d) is higher than available pairs (%d). All will be used.', ...
            maxRobots, nTotal);
    maxRobots = nTotal;
end
rng('shuffle');
selIdx = randperm(nTotal, maxRobots);
%selIdx = 200:1000;
selectedPts = robotPts(selIdx);
fprintf('Selected %d of %d pairs of start/goal.\n', maxRobots, nTotal);
total_geometric_penalty_added = 0;
% --- Topology (1 = obstacle, 0 = Free) ---
T = build_topology_from_B(B);
fullAdj = T.adj;
N = size(fullAdj, 1);
if isfield(T, 'free') && ~isempty(T.free)
    freeIdx = T.free(:);
else
    error('Wrong T structure!');
end
% --- Subgraph only free (sparse & binary) ---
adj = fullAdj(freeIdx, freeIdx);
adj = spones(adj);              % Original, unblocked topology
adj = tril(adj, -1) + tril(adj, -1)';
initial_adj = adj; % Preserve the clean, original topology

[Pre, Post] = construct_PN(adj);
[nplaces, ~] = size(Pre);
invMap = zeros(N, 1);
invMap(freeIdx) = 1:numel(freeIdx);
fwdMap = freeIdx;

% =================================================================
%              *** NON-DESTRUCTIVE FEASIBILITY CHECK ***
% =================================================================
fprintf(1,'\n--- Starting Individual Unconstrained Feasibility Check ---\n');

% 1. IDENTIFY ALL START/GOAL NODES
all_S_Idx = zeros(1, maxRobots);
all_G_Idx = zeros(1, maxRobots);
for i = 1:maxRobots
    [~, ~, S_orig, G_orig] = initial_marking(selectedPts{i}, B, invMap, nplaces); 
    all_S_Idx(i) = S_orig; 
    all_G_Idx(i) = G_orig;
end

% 2. CHECK SOLVABILITY ON THE ORIGINAL MAP
unsolvable_agents = [];
G_orig = graph(initial_adj); 

for currentRobot = 1:maxRobots
    S_curr = all_S_Idx(currentRobot);
    G_curr = all_G_Idx(currentRobot);
    
    % Solve path on the original, unconstrained graph
    [~, path_cost] = shortestpath(G_orig, S_curr, G_curr);
    
    if isinf(path_cost)
        fprintf('Agent %d is UNSOLVABLE (S:%d, G:%d) due to map obstacles.\n', currentRobot, S_curr, G_curr);
        unsolvable_agents = [unsolvable_agents, currentRobot];
    end
end

% 3. SUMMARY AND CONTINGENCY
num_unsolvable = numel(unsolvable_agents);
fprintf('--------------------------------------------------\n');

if num_unsolvable > 0
    fprintf('**FEASIBILITY FAILURE**: %d out of %d agents are geometrically unsolvable.\n', num_unsolvable, maxRobots);
    fprintf('The script will now exit.\n');
    fprintf('--------------------------------------------------\n');
    return; % Exit the script
else
    fprintf('**FEASIBILITY SUCCESS**: All %d agents are individually solvable.\n', maxRobots);
    fprintf('The prioritized planning phase will now proceed on the original problem.\n');
    fprintf('--------------------------------------------------\n');
end

% =================================================================
%            *** START OF PRIORITIZED PLANNING ***
% =================================================================

fprintf(1,'Start solving the MAPF problem using prioritized planning\n');

%%% NEW STEP: SET THE PRIORITY ORDER %%%
% Define your priority heuristic here:
PRIORITY_HEURISTIC = 'random'; % Options: 'random', 'shortest_first', 'longest_first', 'original', 'conflicting_first', 'conflicting_last'

% Generate the prioritized list of agent indices
priority_order = get_priority_order(all_S_Idx, all_G_Idx, initial_adj, PRIORITY_HEURISTIC);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nRobots = 1; % This counter now tracks the position in the priority_order list (1 to maxRobots)
paths = cell(1,maxRobots);
times = zeros(1,maxRobots);
trans_collisions={}; 
time_limit = 1200; 
not_solved = [];
collisions = [];

% --- Initialization for A* / Dijkstra ---
edge_costs = initial_adj; % Reset to the clean, original topology
traj_anterior_places = []; 
plot_map(B);

total_timer_start = tic; 

while nRobots <= maxRobots
    
    agent_idx = priority_order(nRobots);

    % Use the original start and goal indices for the current prioritized agent
    idxStart = all_S_Idx(agent_idx);
    idxGoal = all_G_Idx(agent_idx);
    
    if ~isempty(traj_anterior_places)
        temp = length(traj_anterior_places);
        
        for j = 1 : temp
            visited_place = traj_anterior_places(j);
            incoming_neighbors = find(initial_adj(:, visited_place));
            
            for k = 1 : length(incoming_neighbors)
                u = incoming_neighbors(k);
                % Apply dynamic cost inflation
                edge_costs(u, visited_place) = edge_costs(u, visited_place) + temp;
                edge_costs(visited_place, u) = edge_costs(visited_place, u) + temp;
                % edge_costs(u, visited_place) = 1; % Edge cost remains 
                % edge_costs(visited_place, u) = 1; % Edge cost remains 1
            end
        end
    end
    
    solver_timer_start = tic; 
    
    G = digraph(edge_costs); 
    [path_nodes, path_cost] = shortestpath(G, idxStart, idxGoal);
   
    time_solve = toc(solver_timer_start); 
    
    if time_solve > time_limit
        fprintf('Solving the path problem takes more than %i seconds! Breaking.\n',time_limit);
        break;
    else
        if isempty(path_nodes) || isinf(path_cost)
            
            fprintf('Geometric penalty block for agent %d (orig: %d). Reverting to unconstrained path.\n', nRobots, agent_idx);
            not_solved = [not_solved agent_idx];
            % Re-solve using the clean, initial topology (initial_adj)
            G_revert = graph(initial_adj); 
            [path_nodes, path_cost] = shortestpath(G_revert, idxStart, idxGoal);
            
            if isempty(path_nodes) || isinf(path_cost)
                % If it still fails on the clean map, it's a true geometric failure.
                fprintf('Agent %d (orig: %d) truly UNSOLVABLE. Skipping.\n', nRobots, agent_idx);
                not_solved = [not_solved agent_idx];
                nRobots = nRobots + 1;
                continue;
            end
            
        else
            % --- (Conflict Flagging) ---
            path_length = numel(path_nodes) - 1; 
            
            if (path_cost > path_length)
                collisions(end+1) = agent_idx; % Store the original agent index
                
                problem_nodes = [];
                for i = 1:path_length
                    u = path_nodes(i);
                    v = path_nodes(i+1);
                    if edge_costs(u, v) > 1 
                        problem_nodes(end+1) = v; 
                    end
                end
                trans_collisions{numel(collisions)} = unique(problem_nodes); 
            end
        end 
    end
    
    path_length = numel(path_nodes) - 1; 
    
    if (path_cost > path_length)
        collisions(end+1) = agent_idx; 
        
        problem_nodes = [];
        for i = 1:path_length
            u = path_nodes(i);
            v = path_nodes(i+1);
            if edge_costs(u, v) > 1 
                problem_nodes(end+1) = v; 
            end
        end
        trans_collisions{numel(collisions)} = unique(problem_nodes); 
    end
    
    % --- Update for next Robot ---
    traj_anterior_places = path_nodes; 
    paths{agent_idx} = path_nodes; % Store path at the original agent index
    
    goal_node = path_nodes(end);
    
    incoming_neighbors = find(initial_adj(:, goal_node));
    for k = 1 : length(incoming_neighbors)
        u = incoming_neighbors(k);
        edge_costs(u, goal_node) = Inf; 
        edge_costs(goal_node, u) = Inf; 
    end
    
    L_geometric = numel(path_nodes) - 1; % Path length (time steps)
    if L_geometric > 0
        penalty_added = path_cost - L_geometric;
        total_geometric_penalty_added = total_geometric_penalty_added + penalty_added;
    else
        penalty_added = 0;
    end

    times(agent_idx) = time_solve; % Store time at the original agent index
    nRobots = nRobots + 1;
end

total_runtime = toc(total_timer_start);
fprintf(1,'\n--- Starting Collision Avoidance ---\n');

sync_time_start = tic;
[synched_paths, total_pos_collisions] = local_controller(paths, priority_order, nplaces);
sync_runtime = toc(sync_time_start);
% --- END SYNCHRONIZATION ---

% --- Verification Block ---
wait_actions_inserted = 0;
total_synced_time = 0;

for i = 1:maxRobots
    planned_path = paths{i};
    synched_path = synched_paths{i};
    
    if ~isempty(planned_path) && ~isempty(synched_path)
        L_planned = length(planned_path);
        L_synced = length(synched_path);
        
        delays = L_synced - L_planned;
        
        if delays > 0
            fprintf('Agent %d: %d wait action(s) inserted. Length increased from %d to %d.\n', ...
                    i, delays, L_planned, L_synced);
        end
        wait_actions_inserted = wait_actions_inserted + delays;
        total_synced_time = total_synced_time + L_synced;
    end
end

fprintf('\nSummary of Collision Resolution:\n');
fprintf('Total Collisions Flagged by Controller (total_pos_collisions): %d\n', total_pos_collisions);
fprintf('Total Wait Actions Inserted (Verified Length Check): %d\n', wait_actions_inserted);
fprintf('Total Time Steps (Sum of all agent path lengths): %d\n', total_synced_time);

% Check if the internal counter matches the verified length check
if total_pos_collisions == wait_actions_inserted
    fprintf('SUCCESS: Internal counter matches verified path length increase.\n');
else
    fprintf('WARNING: Internal counter and verified path length increase DO NOT match. Check synchronization logic.\n');
end
% --------------------------

total_cost_soc = 0;
total_geometric_cost = 0; % The cost without any conflicts (base cost)

for i = 1:maxRobots
    L_synced = 0;
    L_planned = 0;

    synched_path = synched_paths{i};
    
    planned_path = paths{i}; 

    if ~isempty(synched_path)
        L_synced = length(synched_path) - 1; % Total moves + waits
        total_cost_soc = total_cost_soc + L_synced;
    end
    
    if ~isempty(planned_path)
        L_planned = length(planned_path) - 1; % Total moves (geometric cost)
        total_geometric_cost = total_geometric_cost + L_planned;
    end
end

% --- DISPLAY RESULTS ---
total_delay = total_cost_soc - total_geometric_cost;

fprintf('\n=== RESULTS SUMMARY: TOTAL COST ===\n');
fprintf('1. Total Synchronized Cost (SOC): %d\n', total_cost_soc);
fprintf('2. Base Geometric Cost (No Conflicts): %d\n', total_geometric_cost);
fprintf('3. Total Delay (SOC - Base Cost): %d\n', total_delay);
fprintf('===================================\n');

sched_points = plotXY(B, fwdMap, synched_paths);

num_bloqueos = numel(not_solved);

fprintf('\n--------------------------------------------------\n');
fprintf('Total solving time for %d robots: %s seconds\n', maxRobots, num2str(total_runtime));
fprintf('Sum of individual *solver* times: %s seconds\n', num2str(sum(times)));
fprintf('Syncro time: %s seconds\n', num2str(sync_runtime)); 
fprintf('BLOQUEOS (prevenidos.): %d\n', num_bloqueos);
fprintf('TOTAL COLLSISIONS RESOLVED (Wait Actions): %d\n', total_pos_collisions); 
fprintf('--------------------------------------------------\n');
plot_trajectories(B, sched_points,1,1)

animation_speed = 0.01;
animate_trajectories(B, fwdMap, synched_paths, 3, animation_speed);
% Save results
save("sim.mat","synched_paths", "paths","times","not_solved","collisions","trans_collisions","Pre","Post", "total_runtime", "sync_runtime");