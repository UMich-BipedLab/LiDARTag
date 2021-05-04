function tag = loadAprilTagFamily()
    tag.family = 4; % LiDARTag family
    tag.black_border = 1; 
    tag.white_border = 1;
    tag.grid_size = 0.5; % size of each grid
    tag.num_bit = tag.family + 2*tag.black_border + 2*tag.white_border; % bit on a side
    tag.size = tag.grid_size * tag.num_bit;
end