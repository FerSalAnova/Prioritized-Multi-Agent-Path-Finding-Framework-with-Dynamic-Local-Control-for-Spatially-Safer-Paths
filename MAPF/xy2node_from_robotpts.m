function node = xy2node_from_robotpts(pt_xy, B, invMap)
% pt_xy = [x y] (0-based, cartesiano, como en robotPts)
    col = pt_xy(1) + 1;   % MATLAB 1-based
    row = pt_xy(2) + 1;
    iOriginal = sub2ind(size(B), row, col);

    if B(row,col) == 1
        error('El punto (%d,%d) cae en obstáculo.', pt_xy(1), pt_xy(2));
    end

    node = invMap(iOriginal);
    if node == 0
        error('El punto no pertenece al grafo reducido (¿obstáculo?).');
    end
end