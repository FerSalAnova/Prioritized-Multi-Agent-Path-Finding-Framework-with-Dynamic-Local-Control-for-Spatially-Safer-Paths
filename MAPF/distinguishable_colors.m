function [colors] = distinguishable_colors(n_colors)
% DISTINGUISHABLE_COLORS Generates a colormap of N visually distinct colors, 
% prioritizing high saturation and brightness.

    % --- 1. Define Candidate Colors in HSV Space ---
    % H: Hue (0 to 1) - spread around the color wheel.
    % S: Saturation (0 to 1) - avoid low saturation (grays).
    % V: Value/Brightness (0 to 1) - avoid low value (darks).
    
    % Generate a high-resolution grid of possible colors
    n_hue = 30;
    n_sat = 3;
    n_val = 4;
    
    % Define the target ranges for vibrant colors:
    hues = linspace(0, 1, n_hue + 1)'; hues(end) = [];
    sats = linspace(0.5, 1, n_sat); % Only medium to high saturation
    vals = linspace(0.7, 1, n_val); % Only bright values

    % Create the full candidate list (HSV coordinates)
    candidate_hsv = zeros(n_hue * n_sat * n_val, 3);
    count = 1;
    for h = 1:n_hue
        for s = 1:n_sat
            for v = 1:n_val
                candidate_hsv(count, :) = [hues(h), sats(s), vals(v)];
                count = count + 1;
            end
        end
    end
    
    % Remove unused rows
    candidate_hsv = candidate_hsv(1:count-1, :);
    
    % Convert to RGB
    candidate_rgb = hsv2rgb(candidate_hsv);
    
    if n_colors <= size(candidate_rgb, 1)
        
        % --- 2. Select the most distinguishable set of N colors ---
        
        % Simplified greedy selection: Pick the first color, then repeatedly 
        % pick the color that is farthest (in RGB space) from the already chosen colors.
        
        % Initialize the selected colors array
        colors = zeros(n_colors, 3);
        
        % Choose the first color randomly from the candidates
        rng('shuffle');
        colors(1, :) = candidate_rgb(randi(size(candidate_rgb, 1)), :);
        
        for i = 2:n_colors
            % Calculate the minimum distance of each candidate to the chosen colors
            D = zeros(size(candidate_rgb, 1), 1);
            for j = 1:size(candidate_rgb, 1)
                % Distance in squared Euclidean space
                distances = sum(bsxfun(@minus, candidate_rgb(j, :), colors(1:i-1, :)).^2, 2);
                D(j) = min(distances);
            end
            
            % Select the candidate with the maximum minimum distance
            [~, idx] = max(D);
            colors(i, :) = candidate_rgb(idx, :);
        end
    else
        % If n_colors is huge, just return the full, sorted candidate set
        warning('Requested number of colors (%d) exceeds optimal candidates. Returning full candidate set.', n_colors);
        colors = candidate_rgb;
    end
end