function [x, y] = lin2xy_cart(iLinear, B)
% Convierte índice lineal de B a coordenadas cartesianas 0-based:
% x = columna-1, y = fila-1, con (0,0) abajo-izquierda.
    [row, col] = ind2sub(size(B), iLinear);
    x = col - 1;
    y = row - 1;
end
