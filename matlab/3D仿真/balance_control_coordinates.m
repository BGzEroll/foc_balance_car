function controlRaw=balance_control_coordinates(worldRaw)
% 报告用：N×14世界测量，XYZ右/前/上 -> 控制分析坐标前/左/上。
% 四元数同时转换父、子坐标基底，避免给偏航添加90度偏置。
controlRaw=worldRaw(:,[1 4 5 2 3 6 7 8 10 9 11 13 12 14]);
controlRaw(:,[4 5 10 13])=-controlRaw(:,[4 5 10 13]);
end
