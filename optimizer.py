# I'll be using Google OR-TOOLS for this
from ortools.linear_solver import pywraplp
import itertools
import ortools

# Instantiating our solver
solver = pywraplp.Solver("Mindustry Mining Solver",
                        pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

### Importing our problem. ###
# Problem must be rectangular, can change later
# By default you can output in any direction, will change later

# 1=o=ore, 0=.=nothing, -1=x=inaccessible
char_map = {"x":-1, ".":0, "o":1}
ore_map = []

# Reading and formatting the file
with open("problems/problem_ex1.txt") as f:
    for line in f:
        line = line.strip()
        map_line = tuple(map(lambda x: char_map[x], line))
        ore_map.append(map_line)

# Is this map rectangular?
if not all([len(line) == len(ore_map[0]) for line in ore_map]):
    print("Sorry, this map isn't rectangular")
    exit()
    

### Configuration Variables ###
max_m_output = 5
max_b_output = 5

### Setting up our iterators ###
X = range(0, len(ore_map[0]))
max_x = len(ore_map[0])-1
Y = range(0, len(ore_map))
max_y = len(ore_map)-1

# faces
F = ["north", "east", "south", "west"]

### Variable Definitions ###
## Machines
M = {}
M_out = {}
M_out_constraints = {}

# What spots are adjacent to the machine on (x,y)?
def adjacent_to_m(x,y):
    adj_list = []

    for j in [y-1, y+2]:
        for i in [x, x+1]:
            if 0 <= i <= max_x and 0 <= j <= max_y:
                adj_list.append((i,j))
    
    for j in [y, y+1]:
        for i in [x-1, x+2]:
            if 0 <= i <= max_x and 0 <= j <= max_y:
                adj_list.append((i,j))

    return adj_list

# How much ore is accessible to the machine on (x,y)?
def ore_for_m(x,y):
    return sum([ore_map[i][j] == 1 for i in range(y, y+1) for j in range(x, x+1)])

# Machine loop
for (x,y) in itertools.product(X,Y):
    # Can we actually place this machine?
    if x < max_x and y < max_y and all(ore_map[j][i] != -1 for j in [y, y+1] for i in [x, x+1]):
        # Machine placement variables
        M[x,y] = solver.IntVar(0.0, 1.0, "Machine x: {} y: {}".format(x,y))
        
        # Machine output variables
        for (i,j) in adjacent_to_m(x,y):
            M_out[x,y,i,j] = solver.NumVar(0, ore_for_m(x,y)*max_m_output, "Machine Output x: {} y: {} i: {} j: {}".format(x,y,i,j))
            
            # We can't output if we don't exist
            M_out_constraints[x,y,i,j] = solver.Add(M_out[x,y,i,j] <= ore_for_m(x,y)*max_m_output*M[x,y])



## Belts
B = {}
B_out = {}
B_out_constraints = {}

# Belt loop
for (x,y) in itertools.product(X,Y):
    if ore_map[y][x] != -1:
        for f in F:
            # Belt placement variables
            B[x,y,f] = solver.IntVar(0.0, 1.0, "Belt x: {} y: {} f: {}".format(x,y,f))
            B_out[x,y,f] = solver.NumVar(0.0, max_b_output, "Belt Output x: {} y: {} f: {}".format(x,y,f))

            # We can't output if we don't exist
            B_out_constraints = solver.Add(B_out[x,y,f] <= B[x,y,f] * max_b_output)


### Auxiliary Functions and lists ###

## Which output variables feed into a square?
def feeds_into(x,y):
    # Belts
    feeders = [(x-1, y, "east"), (x+1, y, "west"), (x, y-1,"south"), (x, y+1,"north")]
    feeders = [B_out[e] for e in feeders if e in B]
    
    # Machines
    for j in [y+1, y-2]:
        for i in [x, x-1]:
            if (i,j) in M:
                feeders.append(M_out[i,j,x,y])
    
    for j in [y, y-1]:
        for i in [x+1, x-2]:
            if (i,j) in M:
                feeders.append(M_out[i,j,x,y])
    
    return feeders

## Which outputs feed outside?
feeding_outside = []

# belts
for y in Y:
    feeding_outside.append(B_out[0,y,"west"])
    feeding_outside.append(B_out[max_x, y, "east"])

for x in X:
    feeding_outside.append(B_out[x, 0, "north"])
    feeding_outside.append(B_out[x, max_y, "south"])



### Constraints ###
## 4) Belts can't overlap anything
Overlap_Constraints = {(x,y): solver.Add(sum(B[x,y,f] for f in F) <= 1.0 - sum(M[i,j] for i in [x,x-1] for j in [y, y-1] if (i,j) in M))
                        for x in X for y in Y}

## 6)
Belt_Conservation = {(x,y): solver.Add(sum(B_out[x,y,f] for f in F) == sum(feeds_into(x,y)))
                    for x in X for y in Y}

## 7) Conservation of flow
Conservation_Constraint = solver.Add(sum(m for m in M_out.values()) == sum(feeding_outside))


### What do we optimize ###
solver.Maximize(sum(b for b in feeding_outside))
status = solver.Solve()


if status == pywraplp.Solver.OPTIMAL:
    ### Printing ###
    print('Number of variables =', solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())

    # Printing the grid
    grid = [["." for x in X] for y in Y]

    # Adding machines
    for (x,y) in itertools.product(X,Y):
        if (x,y) in M and M[x,y].solution_value() > 0.5:
            for i in [x,x+1]:
                for j in [y,y+1]:
                    grid[j][i] = "M"

    # Adding Belts
    belt_char_map = {"north": "^", "east": ">", "south": "v", "west": "<"}
    for (x,y) in itertools.product(X,Y):
        for f in F:
            if B[x,y,f].solution_value() > 0.5: 
                grid[y][x] = belt_char_map[f]
    # Grid printing
    for line in grid:
        for char in line:
            print(char,end="")
        print()

else:
    print("No Optimal Assignment for this problem")
