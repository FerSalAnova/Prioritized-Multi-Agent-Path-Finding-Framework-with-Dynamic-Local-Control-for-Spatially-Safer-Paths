function [m0, mF, idxStart, idxGoal] = initial_marking(robot, B, invMap, numPlaces)
% INITIAL_MARKING
% Calcula el marcado inicial y el marcado final (deseado) de la red de Petri
% para un robot dado.
%
% INPUTS:
%   robot      - [x_start y_start; x_goal y_goal] (0-based, cartesiano)
%                o celda que contenga esa matriz
%   B          - matriz binaria (1 = obstáculo, 0 = libre)
%   invMap     - vector tal que invMap(iOriginal) = iReducido (0 si obstáculo)
%   numPlaces  - número de lugares (== size(adj,1))
%
% OUTPUTS:
%   m0         - marcado inicial  (vector columna con un único 1 en el lugar inicial)
%   mF         - marcado final    (vector columna con un único 1 en el lugar objetivo)
%   idxStart   - índice de lugar inicial (en adj)
%   idxGoal    - índice de lugar objetivo (en adj)
%
% Ejemplo:
%   p = robotPts{1};
%   [m0, mF] = initial_marking(p, B, invMap, size(adj,1));

    % Aceptar celda o matriz
    if iscell(robot)
        robot = robot{1};
    end
    robot = double(robot);

    % Validación básica
    if ~ismatrix(robot) || size(robot,2) ~= 2 || size(robot,1) < 2
        error('robot debe ser [x_start y_start; x_goal y_goal] con dos filas y dos columnas.');
    end

    % --- Coordenadas (0-based) ---
    start_xy = robot(1,:);   % [x y]
    goal_xy  = robot(2,:);   % [x y]

    % --- Conversión a índices MATLAB (1-based) ---
    [H, W] = size(B);

    % START
    colS = start_xy(1) + 1;
    rowS = start_xy(2) + 1;
    if colS < 1 || colS > W || rowS < 1 || rowS > H
        error('Start fuera de rango: x=%d, y=%d', start_xy(1), start_xy(2));
    end
    if B(rowS, colS) == 1
        error('La posición inicial (%d,%d) está en un obstáculo.', start_xy(1), start_xy(2));
    end
    iOrgS   = sub2ind([H, W], rowS, colS);
    idxStart = invMap(iOrgS);
    if idxStart == 0
        error('No se encontró el lugar inicial en el grafo reducido (invMap=0).');
    end

    % GOAL
    colG = goal_xy(1) + 1;
    rowG = goal_xy(2) + 1;
    if colG < 1 || colG > W || rowG < 1 || rowG > H
        error('Goal fuera de rango: x=%d, y=%d', goal_xy(1), goal_xy(2));
    end
    if B(rowG, colG) == 1
        error('La posición objetivo (%d,%d) está en un obstáculo.', goal_xy(1), goal_xy(2));
    end
    iOrgG   = sub2ind([H, W], rowG, colG);
    idxGoal = invMap(iOrgG);
    if idxGoal == 0
        error('No se encontró el lugar objetivo en el grafo reducido (invMap=0).');
    end

    % --- Marcados ---
    m0 = zeros(numPlaces, 1);
    mF = zeros(numPlaces, 1);
    m0(idxStart) = 1;
    mF(idxGoal)  = 1;
end
