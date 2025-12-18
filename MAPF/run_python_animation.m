function run_python_animation(B, robotPts, sched_points, pyFile)
%RUN_PYTHON_ANIMATION Llama a la clase Animation (Python) desde MATLAB.
%   B            : mapa binario HxW (1=obstáculo, 0=libre) EN CARTESIANO (flipud ya hecho).
%   robotPts     : cell R x 1, cada {r} = [sx sy; gx gy] en CARTESIANO (tu loader).
%   sched_points : cell R x 1, cada {r} = [T x 2] (x,y) paso-sincrónico en CARTESIANO.
%   pyFile       : ruta al fichero .py que contiene la clase Animation (p.ej. 'animation_icbs.py').


%python -m pip install numpy matplotlib

    arguments
        B {mustBeNumeric, mustBeNonempty}
        robotPts cell
        sched_points cell
        pyFile {mustBeTextScalar}
    end

    % ------------- 1) Asegurar Python en MATLAB -------------
    % Ajusta la ruta si quieres forzar una versión concreta:
    % pyenv('Version','C:\Users\TuUsuario\AppData\Local\Programs\Python\Python311\python.exe');
    pe = pyenv;
    fprintf('[pyenv] Python: %s (Version=%s)\n', pe.Executable, pe.Version);

    % Añadir a sys.path la carpeta del .py si hace falta
    [pyFolder, pyName, ~] = fileparts(pyFile);
    if ~isempty(pyFolder)
        if count(py.sys.path, string(pyFolder)) == 0
            insert(py.sys.path, int64(0), string(pyFolder));
        end
    end
    % Importar el módulo
    mod = py.importlib.import_module(pyName);
    py.importlib.reload(mod);  % por si editas el .py

    % ------------- 2) Convertir datos a lo que espera Python -------------
    % IMPORTANTÍSIMO: tu B en MATLAB está en CARTESIANO (flipud hecho).
    % La clase Python hace: my_map -> np.transpose -> flip eje x, y luego pinta.
    % Esa clase espera "starts/goals/paths" en convención imagen (fila,col) con (0,0) ARRIBA-IZQUIERDA.
    % Por lo tanto, convertimos:
    %   - B_img = flipud(B)  -> vuelve a "imagen".
    %   - Para cada punto (x,y) cartesiano -> (row = H-1-y, col = x).
    [H, W] = size(B);
    B_img = flipud(B);                 % volver a "imagen"
    my_map_np = py.numpy.array(uint8(B_img));  % 1=obstáculo

    % starts & goals (fila, col) 0-based "imagen"
    R = numel(robotPts);
    starts_py = py.list;
    goals_py  = py.list;
    for r = 1:R
        P = robotPts{r};          % [sx sy; gx gy] en CARTESIANO
        sx = P(1,1);  sy = P(1,2);
        gx = P(2,1);  gy = P(2,2);
        row_s = int64(H - 1 - sy);   col_s = int64(sx);
        row_g = int64(H - 1 - gy);   col_g = int64(gx);
        starts_py.append(py.tuple({row_s, col_s}));
        goals_py.append(py.tuple({row_g, col_g}));
    end

    % paths (secuencia de (fila,col) por paso). Tu sched_points está en CARTESIANO.
    paths_py = py.list;
    for r = 1:R
        XY = sched_points{r};             % [T x 2] (x,y)
        if isempty(XY)
            paths_py.append(py.list);
            continue;
        end
        % Convertir cada (x,y) a (row,col) 0-based "imagen"
        % Si están en centros (x.5, y.5), redondeamos al nodo más cercano:
        rc_list = py.list;
        for k = 1:size(XY,1)
            x = XY(k,1);  y = XY(k,2);
            row = int64(round(H - 1 - y));
            col = int64(round(x));
            rc_list.append(py.tuple({row, col}));
        end
        paths_py.append(rc_list);
    end

    % ------------- 3) Crear animación y mostrar/guardar -------------
    anim = mod.Animation(my_map_np, starts_py, goals_py, paths_py);
    % Mostrar en una ventana interactiva:
    anim.show();

    % O guardar a vídeo (descomenta si quieres):
    % anim.save('anim_icbs.mp4', int64(1));  % speed=1 (usa FFmpeg si lo tienes)
end
