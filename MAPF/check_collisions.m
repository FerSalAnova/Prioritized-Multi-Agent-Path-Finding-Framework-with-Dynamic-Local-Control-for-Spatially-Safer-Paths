function conflicts = check_collisions(paths)
% Simula los caminos paso a paso y reporta colisiones.
%
% ENTRADA:
%   paths: {1xR} celda, donde paths{r} es un vector de ÍNDICES de nodos (lugares).
%
% SALIDA:
%   conflicts: Celda con todos los reportes de colisión.

fprintf('--- Iniciando simulación de colisiones en MATLAB ---\n');

R = numel(paths);
max_T = 0;
for r = 1:R
    max_T = max(max_T, numel(paths{r}));
end

conflicts = {}; % Celda para almacenar los reportes

% --- 1. Bucle a través de cada paso de tiempo ---
for t = 1:max_T
    
    % --- 2. Bucle a través de todos los pares únicos de robots ---
    for r1 = 1:R
        for r2 = (r1 + 1):R
            
            % --- 3. Obtener la posición (índice de nodo) de cada robot en el tiempo t ---
            
            % Posición del robot r1
            path1 = paths{r1};
            if isempty(path1)
                continue; % Saltar si el robot no tiene camino
            end
            if t > numel(path1)
                pos1_t = path1(end); % El robot se queda en su meta
            else
                pos1_t = path1(t);
            end

            % Posición del robot r2
            path2 = paths{r2};
            if isempty(path2)
                continue; % Saltar si el robot no tiene camino
            end
            if t > numel(path2)
                pos2_t = path2(end); % El robot se queda en su meta
            else
                pos2_t = path2(t);
            end

            % --- 4. Comprobar Colisión de Nodos (Vértices) ---
            % Dos robots en el mismo nodo al mismo tiempo
            if pos1_t == pos2_t
                report = sprintf('COLISIÓN DE NODO en t=%d: Robot %d y Robot %d en el nodo %d.', ...
                                 t, r1, r2, pos1_t);
                % fprintf(2, '%s\n', report); % Imprimir en rojo (error)
                conflicts{end+1} = report;
                continue; % Ya han chocado, no es necesario comprobar el 'swap'
            end

            % --- 5. Comprobar Colisión de 'Swap' (Aristas) ---
            % Ocurre entre el tiempo t y t+1
            if t < max_T
                % Obtener pos en t+1 para r1
                if (t + 1) > numel(path1)
                    pos1_t_plus_1 = path1(end);
                else
                    pos1_t_plus_1 = path1(t + 1);
                end
                
                % Obtener pos en t+1 para r2
                if (t + 1) > numel(path2)
                    pos2_t_plus_1 = path2(end);
                else
                    pos2_t_plus_1 = path2(t + 1);
                end

                % Comprobar si intercambian lugares
                if (pos1_t == pos2_t_plus_1) && (pos1_t_plus_1 == pos2_t)
                    report = sprintf('COLISIÓN DE SWAP en t=%d->%d: Robot %d (%d->%d) y Robot %d (%d->%d).', ...
                                     t, t+1, r1, pos1_t, pos1_t_plus_1, r2, pos2_t, pos2_t_plus_1);
                    % fprintf(2, '%s\n', report);
                    conflicts{end+1} = report;
                end
            end
        end
    end
end

if isempty(conflicts)
    fprintf('--- Simulación Completa: No se encontraron colisiones. ---\n');
else
    fprintf('--- Simulación Completa: Se encontraron %d colisiones. ---\n', numel(conflicts));
end
end