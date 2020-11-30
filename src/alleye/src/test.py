import numpy as np


if __name__ == "__main__":
    
    max_bound = (100.,100.)
    obstacles = [(50.,10.)]
    width = 10
    box_size = int(max_bound[0]/width)
    world = np.zeros((width, width))

    obs_bb = (20.,20.)

    for x,y in obstacles:
        
        bb_tl = (x-obs_bb[0], y-obs_bb[1])
        bb_br = (x+obs_bb[0], y+obs_bb[1])
        # print(bb_tl)
        # print(bb_br)

        # i = int(round(x/box_size))
        # j = int(round(y/box_size))
        # print("(%f,%f) -> (%d,%d)" % (x,y,i,j))

        i_min = int(round(bb_tl[1]/box_size))
        i_max = int(round(bb_br[1]/box_size))
        j_min = int(round(bb_tl[0]/box_size))
        j_max = int(round(bb_br[0]/box_size))

        print(i_min, j_min)
        print(i_max, j_max)

        world[i_min:i_max, j_min:j_max] = 1
     
    print(world)
