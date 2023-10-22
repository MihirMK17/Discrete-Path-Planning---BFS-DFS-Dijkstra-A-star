import matplotlib.pyplot as plt
from mazemods import *
"""
Number of rows and cols in the grid
"""
x = 5
y = 5
n, m = 150//x, 200//y
"""
The obstacles are mentioned in the variable O
"""
O = [[0//x,149//x,0//y,5//y], [0//x,5//x,0//y,199//y], [0,149//x,194//y,199//y], [144//x,149//x,0//y,199//y], # The boundary
     [0,80//x,180//y,199//y], # The curb/ pavement
     [5//x,25//x,120//y,160//y], [40//x,60//x,25//y,65//y], [124//x,144//x,85//y,125//y], # The parked cars
     # [85,110,15,25], [85,110,65,75], # Parked bikes
     [55//x,95//x,80//y,140//y], # Building
     [70//x,80//x,0,80//y] # Divider
     ]
# #
# maze(n,m,O)
# plt.show()
