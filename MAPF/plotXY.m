function sched_points = plotXY(B, fwdMap, sched_paths, varargin)
% PLACES2XY  Convierte secuencias de lugares (índices de "places") a (x,y) cartesianos.
%
% USO:
%   sched_points = places2xy(B, fwdMap, sched_paths)
%   sched_points = places2xy(B, fwdMap, sched_paths, 'UseCenters', true, 'BIsCartesian', true)
%
% ENTRADAS:
%   B            : matriz binaria HxW del mapa (1=obstáculo, 0=libre).
%                  Si 'BIsCartesian'==true, B ya está volteada (y hacia arriba).
%   fwdMap       : vector tal que fwdMap(placeIdx) = índice lineal en B (== freeIdx).
%   sched_paths  : celda 1xR; sched_paths{r} = [p1 p2 ... pT] (índices de lugar por paso).
%
% OPCIONES (pares 'Nombre', valor):
%   'UseCenters'  : true (centros de celda) | false (esquinas). Default: true.
%   'BIsCartesian': true si B ya está en coordenadas cartesianas (flipud hecho).
%                   false si B sigue en convención de imagen (0,0 arriba-izquierda). Default: true.
%
% SALIDA:
%   sched_points : celda 1xR; sched_points{r} = [T x 2], con (x,y) cartesianos por paso.
%
% NOTAS:
%   - Si usas el loader que hace B = flipud(B), pasa 'BIsCartesian', true (por defecto).
%   - Asegúrate de que fwdMap (freeIdx) corresponde al MISMO B que usaste para construir el grafo/PN.

% ---- Parámetros
p = inputParser;
p.addParameter('UseCenters', true, @(x)islogical(x)&&isscalar(x));
p.addParameter('BIsCartesian', true, @(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});
useCenters   = p.Results.UseCenters;
BIsCartesian = p.Results.BIsCartesian;

[H, W] = size(B); %#ok<NASGU>

% place -> índice lineal en B
linIdx = fwdMap(:);
[rowAll, colAll] = ind2sub(size(B), linIdx);

% Construir tablas (x,y) por placeIdx, según orientación de B
if BIsCartesian
    % ✅ B ya volteado (y crece hacia arriba). Usar fila/col tal cual.
    if useCenters
        xAll = double(colAll) - 0.5;
        yAll = double(rowAll) - 0.5;
    else
        xAll = double(colAll) - 1.0;
        yAll = double(rowAll) - 1.0;
    end
else
    % ❗ B NO está volteado (convención imagen: y hacia abajo). Convertimos aquí.
    if useCenters
        xAll = double(colAll) - 0.5;
        yAll = double(H - rowAll) + 0.5;  % invertir eje y
    else
        xAll = double(colAll) - 1.0;
        yAll = double(H - rowAll);        % invertir eje y
    end
end

% Mapear cada trayectoria programada
R = numel(sched_paths);
sched_points = cell(1, R);
for r = 1:R
    pseq = double(sched_paths{r});
    if isempty(pseq)
        sched_points{r} = zeros(0,2);
    else
        sched_points{r} = [xAll(pseq), yAll(pseq)];
    end
end

% ---- (Opcional) Comprobación de coherencia: todos los lugares deberían ser celdas libres
% Descomenta si quieres verificar:
% for r = 1:R
%     pseq = double(sched_paths{r});
%     if isempty(pseq), continue; end
%     if any(B(sub2ind(size(B), rowAll(pseq), colAll(pseq))) ~= 0)
%         warning('Robot %d: hay pasos que caen en obstáculo. Revisa fwdMap/B.', r);
%     end
% end
end
