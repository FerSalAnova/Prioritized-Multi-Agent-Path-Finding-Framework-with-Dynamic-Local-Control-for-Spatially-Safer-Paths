function plot_simul(B, sched_points, varargin)
% ANIMATE_SCHED_ON_MAP  Dibuja el entorno y anima los robots según las trayectorias programadas.
%
% USO:
%   animate_sched_on_map(B, sched_points)
%   animate_sched_on_map(B, sched_points, 'RobotPts', robotPts, 'Trail', true, 'Dt', 0.02, 'VideoFile','sim.avi')
%
% ENTRADAS:
%   B            - matriz binaria HxW (1=obstáculo, 0=libre), ya en coordenadas cartesianas (flipud hecho).
%   sched_points - celda 1xR; sched_points{r} = [T_r x 2] (x,y por paso)
%
% OPCIONES:
%   'RobotPts'   : cell N x 1 con {[sx sy; gx gy]} para marcar inicios/fin (opcional).
%   'Trail'      : true/false (default true) para mostrar estela.
%   'Dt'         : pausa por frame en segundos (default 0.02).
%   'VideoFile'  : string con nombre del archivo de vídeo (por ejemplo 'sim.avi').

% ---- Parámetros por defecto
p = inputParser;
p.addParameter('RobotPts', [], @(x) iscell(x) || isempty(x));
p.addParameter('Trail', true, @(x) islogical(x) && isscalar(x));
p.addParameter('Dt', 0.02, @(x) isnumeric(x) && isscalar(x) && x>=0);
p.addParameter('VideoFile', [], @(x) (ischar(x) || isstring(x) || isempty(x)));
p.parse(varargin{:});
robotPts   = p.Results.RobotPts;
showTrails = p.Results.Trail;
dt         = p.Results.Dt;
videoFile  = p.Results.VideoFile;

[H, W] = size(B);
R = numel(sched_points);

if R == 0
    warning('sched_points está vacío. Nada que animar.');
    return;
end

% Duración máxima
Tmax = 0;
for r = 1:R
    Tmax = max(Tmax, size(sched_points{r},1));
end

plot_map(B)

pause
% ---- COLORES ----
C = lines(R);

% ---- Marcas de inicio (círculo) y fin (triángulo) ----
hInit = gobjects(1,R);
hGoal = gobjects(1,R);
for r = 1:R
    P = sched_points{r};
    if isempty(P), continue; end
    if ~isempty(robotPts)
        p0 = robotPts{r}(1,:);  % start
        pf = robotPts{r}(2,:);  % goal
    else
        p0 = P(1,:);
        pf = P(end,:);
    end
    hInit(r) = plot(p0(1), p0(2), 'o', 'MarkerSize', 8, 'LineWidth', 1.5, ...
                    'MarkerEdgeColor', C(r,:), 'MarkerFaceColor', 'none');
    hGoal(r) = plot(pf(1), pf(2), '^', 'MarkerSize', 8, 'LineWidth', 1.5, ...
                    'MarkerEdgeColor', C(r,:), 'MarkerFaceColor', 'none');
end

% ---- Robots y trayectorias ----
hRob   = gobjects(1,R);
hTrail = gobjects(1,R);
for r = 1:R
    P = sched_points{r};
    if isempty(P), continue; end
    hRob(r) = plot(P(1,1), P(1,2), '.', 'MarkerSize', 22, 'Color', C(r,:));
    if showTrails
        hTrail(r) = plot(P(1,1), P(1,2), '-', 'LineWidth', 1.0, 'Color', C(r,:));
    end
end
drawnow;

% ---- Configuración de vídeo (opcional) ----
if ~isempty(videoFile)
    vw = VideoWriter(char(videoFile), 'Motion JPEG AVI');
    vw.FrameRate = max(1, round(1/max(1e-4, dt)));
    open(vw);
else
    vw = [];
end

% ---- BUCLE DE ANIMACIÓN ----
for t = 1:Tmax
    for r = 1:R
        P = sched_points{r};
        if isempty(P), continue; end
        idx = min(t, size(P,1)); % mantiene posición final tras llegar
        xy = P(idx,:);
        set(hRob(r), 'XData', xy(1), 'YData', xy(2));
        if showTrails
            set(hTrail(r), 'XData', P(1:idx,1), 'YData', P(1:idx,2));
        end
    end
    drawnow;
    if ~isempty(vw)
        writeVideo(vw, getframe(gcf));
    end
    pause(dt);
end

% ---- Cierre vídeo ----
if ~isempty(vw)
    close(vw);
    fprintf('Vídeo guardado en: %s\n', char(videoFile));
end

% ---- Leyenda opcional ----
legend({'Libre/Obstáculo','Inicio','Meta'}, 'Location','eastoutside');

end
