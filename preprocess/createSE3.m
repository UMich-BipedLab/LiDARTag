function [H] = createSE3(R, position)
%CREATESE3 Creates a 4x4 from a 3x3 rotation matrix and 3x1 position matrix
H = [[R; zeros(1,3)] [position; 1]];
end

