function [B, robotPts, rawScen] = load_map_and_scen(mapFile, scenFile)
% LOAD_MAP_AND_SCEN
%   Reads a MovingAI .map file into a binary matrix (1=obstacle, 0=free),
%   parses the .scen file (start/goal positions),
%   and plots the map using a Cartesian coordinate system (0,0 bottom-left).
%
%   Inputs:
%       mapFile  - path to .map file
%       scenFile - path to .scen file
%
%   Outputs:
%       B         - binary occupancy matrix (1=obstacle, 0=free)
%       robotPts  - cell array, robotPts{i} = [sx sy; gx gy] (Cartesian coords)
%       rawScen   - table with parsed .scen columns
%
% Example:
%   [B, robotPts] = load_map_and_scen('Paris_1_512.map', 'Paris_1_512.map.scen');

if nargin < 3, showK = 0; end

% ---------- Read .map ----------
B = parseMovingAIMap(mapFile);

% Dimensions
[H, W] = size(B);

% ---------- Read .scen ----------
[robotPts, rawScen] = parseMovingAIScen(scenFile);

% Convert coordinates from (0,0 top-left) → (0,0 bottom-left)
for i = 1:numel(robotPts)
    p = robotPts{i};
    % y_cartesian = (height - 1) - y_original
    p(:,2) = (H - 1) - p(:,2);
    robotPts{i} = p;
end

% Flip B vertically to match Cartesian coordinates
B = flipud(B);

%plot_map(B);
%hold on;
% ---------- Overlay sample start/goal pairs ----------
%if showK > 0 && ~isempty(robotPts)
%    K = min(showK, numel(robotPts));
%    for k = 1:K
%        p = robotPts{k};  % [sx sy; gx gy], already Cartesian
%        sx = p(1,1); sy = p(1,2);
%        gx = p(2,1); gy = p(2,2);

        % draw dashed line between start and goal
%        plot([sx gx], [sy gy], 'b--', 'LineWidth', 0.8);

        % mark start (green circle) and goal (red star)
%        plot(sx, sy, 'go', 'MarkerFaceColor','g', 'MarkerSize',6);
%        plot(gx, gy, 'r*', 'LineWidth',1.2, 'MarkerSize',8);
%    end
%    legend({'Free/Obstacle map','Start','Goal'}, 'Location','bestoutside');
%end
%hold off;

end

% =======================================================
function B = parseMovingAIMap(mapFile)
% Reads a MovingAI .map file and returns a binary matrix B
% '@' = obstacle (1)
% others = free (0)

fid = fopen(mapFile,'r');
assert(fid>0, 'Cannot open map file: %s', mapFile);
cleanup = onCleanup(@() fclose(fid));

rows = {};
headerDone = false;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    if headerDone
        rows{end+1} = line; %#ok<AGROW>
    elseif strcmpi(strtrim(line), 'map')
        headerDone = true;
    end
end

assert(~isempty(rows), 'No map data found in file: %s', mapFile);

H = numel(rows);
W = numel(rows{1});
B = zeros(H, W, 'uint8');

for r = 1:H
    rowStr = rows{r};
    if numel(rowStr) ~= W
        error('Inconsistent row width at row %d', r);
    end
    B(r, :) = uint8(rowStr == '@');
end
end

% =======================================================
function [robotPts, scenTbl] = parseMovingAIScen(scenFile)
% Reads a MovingAI .scen file.
% Format: bucket mapName mapW mapH startX startY goalX goalY optimalLength
% Output: robotPts{i} = [sx sy; gx gy] (0-based top-left coords)

fid = fopen(scenFile,'r');
assert(fid>0, 'Cannot open scen file: %s', scenFile);
cleanup = onCleanup(@() fclose(fid));

firstLine = fgetl(fid);
if ~ischar(firstLine)
    error('Empty .scen file: %s', scenFile);
end
if startsWith(strtrim(lower(firstLine)), 'version')
    % skip version line
else
    fseek(fid, 0, 'bof'); % rewind if not version header
end

C = textscan(fid, '%d %s %d %d %d %d %d %d %f', ...
    'Delimiter', {' ','\t'}, 'MultipleDelimsAsOne', true);

scenTbl = table(C{1}, C{2}, C{3}, C{4}, C{5}, C{6}, C{7}, C{8}, C{9}, ...
    'VariableNames', {'bucket','map','mapW','mapH','startX','startY','goalX','goalY','optimalLen'});

N = height(scenTbl);
robotPts = cell(N,1);
for i = 1:N
    robotPts{i} = [scenTbl.startX(i) scenTbl.startY(i); scenTbl.goalX(i) scenTbl.goalY(i)];
end
end
