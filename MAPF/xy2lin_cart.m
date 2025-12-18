function iLinear = xy2lin_cart(x, y, B)
% Convierte coordenadas cartesianas 0-based (x,y) a índice lineal en B.
% x = columna-1, y = fila-1.
    col = x + 1;
    row = y + 1;
    iLinear = sub2ind(size(B), row, col);
end
