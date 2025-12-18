function animate_trajectories(B, fwdMap, synched_paths, figNum, speedFactor)
% ANIMATE_TRAJECTORIES Animates the movement of all robots over time.
% Calls plotXY (or places2xy) for correct coordinate transformation.
%
% B: Map matrix.
% fwdMap: Forward mapping index (reduced index -> linear mesh index).
% synched_paths: Cell array of the final, synchronized paths (place indices).
% figNum: Figure number to use for plotting.
% speedFactor: Controls animation speed (e.g., 0.05 for slow, 0 for fast).

    if nargin < 5
        speedFactor = 0.05; % Default animation speed
    end
    
    % --- 1. Coordinate Conversion using the reliable plotXY function ---
    
    % ASSUMPTION: The plotXY function is available and correctly named in your environment.
    % The output 'sched_points' is a cell array where each cell is an [T x 2] matrix of [x, y] coordinates.
    % We assume default parameters ('UseCenters', true, 'BIsCartesian', true) match your setup.
    sched_points = plotXY(B, fwdMap, synched_paths);

    maxTime = 0;
    nRobots = numel(sched_points);

    % Determine the maximum path length (makespan)
    for i = 1:nRobots
        if ~isempty(sched_points{i})
            maxTime = max(maxTime, size(sched_points{i}, 1));
        end
    end
    
    % 2. Initialize Plot
    figure(figNum);
    clf;
    plot_map(B); % Reuse your map plotting function
    hold on;
    
    % --- Animation Loop Setup ---
    
    % Initialize handles for robot markers and path traces
    robot_handles = zeros(1, nRobots);
    trace_handles = zeros(1, nRobots);
    robot_colors = distinguishable_colors(nRobots); % Assuming this function is available
    
    % Plot initial positions and empty traces
    for r = 1:nRobots
        if ~isempty(sched_points{r})
            current_color = robot_colors(r, :);
            
            % Initial X, Y from the first row of the [T x 2] matrix
            initial_x = sched_points{r}(1, 1);
            initial_y = sched_points{r}(1, 2);
            
            % Plot the robot marker
            robot_handles(r) = plot(initial_x, initial_y, ...
                                   'o', 'MarkerSize', 8, 'MarkerFaceColor', current_color, 'MarkerEdgeColor', 'k');
            
            % Plot the trace (initially just the start point)
            trace_handles(r) = plot(initial_x, initial_y, ...
                                   '-', 'Color', current_color, 'LineWidth', 1.5);
        end
    end

    title(sprintf('Multi-Robot Trajectory Animation (Time: 0/%d)', maxTime - 1));
    xlabel('X Coordinate');
    ylabel('Y Coordinate');
    grid on;
    
    % Iterate through time steps (t=1 is time 0, t=maxTime is the end)
    for t = 1:maxTime
        
        current_time = t - 1;
        title(sprintf('Multi-Robot Trajectory Animation (Time: %d/%d)', current_time, maxTime - 1));
        
        for r = 1:nRobots
            if ~isempty(sched_points{r})
                
                T_r = size(sched_points{r}, 1); % Length of robot r's path
                
                % Determine current position
                if t <= T_r
                    current_x = sched_points{r}(t, 1);
                    current_y = sched_points{r}(t, 2);
                else
                    % Robot has reached its goal and is stationary (at the last position)
                    current_x = sched_points{r}(end, 1);
                    current_y = sched_points{r}(end, 2);
                end
                
                % Update marker position
                if robot_handles(r) ~= 0
                    set(robot_handles(r), 'XData', current_x, 'YData', current_y);
                    
                    % Check if the robot is waiting (by comparing current node index with previous)
                    is_waiting = (t > 1 && t <= T_r && synched_paths{r}(t) == synched_paths{r}(t-1));
                    
                    current_color = robot_colors(r, :);
                    
                    if is_waiting
                        % Robot is waiting, set marker to red
                        set(robot_handles(r), 'MarkerFaceColor', 'r'); 
                    else
                        % Robot is moving, update trace and reset marker color
                        set(trace_handles(r), 'XData', sched_points{r}(1:min(t,T_r), 1), ...
                                               'YData', sched_points{r}(1:min(t,T_r), 2));
                        set(robot_handles(r), 'MarkerFaceColor', current_color); 
                    end
                end
            end
        end
        
        drawnow;
        pause(speedFactor); 
    end
    
    hold off;
end