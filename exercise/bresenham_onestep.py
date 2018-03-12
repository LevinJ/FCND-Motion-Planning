import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = 12, 12




def bres(p1, p2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []
    
    m = (y2 - y1) / (x2 - x1)
    
    line_val = y1
    i = x1
    j = y1
    
    while i < x2:
        cells.append([i, j])
        if line_val + m > j + 1:
            j += 1
        else:
            line_val += m
            i += 1
        
    return np.array(cells)

if __name__ == "__main__":  
    p1 = (0, 0)
    p2 = (7, 5)
    
    cells = bres(p1, p2)
    print(cells)
    
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
    
    
    for q in cells:
        plt.plot([q[0], q[0]+1], [q[1], q[1]], 'k')
        plt.plot([q[0], q[0]+1], [q[1]+1, q[1]+1], 'k')
        plt.plot([q[0], q[0]], [q[1],q[1]+1], 'k')
        plt.plot([q[0]+1, q[0]+1], [q[1], q[1]+1], 'k')
    
    plt.grid()
    plt.axis('equal')
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Integer based Bresenham algorithm")
    plt.show()