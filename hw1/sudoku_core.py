from pysat.formula import CNF
from pysat.solvers import MinisatGH

from ortools.sat.python import cp_model

import clingo

import gurobipy


def neighbors(row:int, col:int, k:int):
    '''
    Gets all neighboring cells of a specific cell
    '''
    s = k**2
    rows = list(range(s))
    cols = list(range(s))
    coords = set()

    # Add cells from the same row and column
    coords.update((row, sub_col) for sub_col in cols)
    coords.update((sub_row, col) for sub_row in rows)

    # Add the cells in the same box
    box_rows = list(range(row//k*k, row//k*k+k))
    box_cols = list(range(col//k*k, col//k*k+k))
    coords.update((sub_row, sub_col) for sub_row in box_rows for sub_col in box_cols)

    # Remove the current cell
    coords.remove((row, col))

    return coords


def propagate(sudoku_possible_values:list, k:int):
    '''
    Propagation function to be used in the recursive sudoku solver
    '''
    s = k**2
    rows = list(range(s))
    cols = list(range(s))

    # List all cell coordinates and cell neighbor coordinates
    coords = [(row, col) for row in rows for col in cols]
    all_neighbors = [[neighbors(row, col, k) for col in cols] for row in rows]

    # Loop over the puzzle until no changes are made anymore
    puzzle_changed = True
    while puzzle_changed:
        puzzle_changed = False
        for row, col in coords:

            # Check if a value is found with 1 possibility
            if len(sudoku_possible_values[row][col]) == 1:
                val = sudoku_possible_values[row][col][0]

                # Remove this possibility from all neighbors,
                # keep track of whether the cell changed anything
                cell_changed = False
                for sub_row, sub_col in all_neighbors[row][col]:
                    try:
                        sudoku_possible_values[sub_row][sub_col].remove(val)
                        cell_changed = True
                    except ValueError:
                        pass

                if cell_changed:
                    puzzle_changed = True
                else:
                    # Cell has no influence, don't try it again next iteration
                    coords.remove((row, col))

    return sudoku_possible_values


def solve_sudoku_SAT(sudoku:list, k:int):
    '''
    Solver that uses SAT encoding
    '''
    s = k**2
    nums = list(range(1, s+1))
    rows = list(range(s))
    cols = list(range(s))

    def index(row:int, col:int, num:int):
        '''
        Get a unique index from the given values
        '''
        return num + s*col + (s**2)*row

    formula = CNF()

    for row in rows:
        for col in cols:

            # All cells should contain a number
            formula.append([index(row, col, num) for num in nums])

            # All cells should ontain only one number
            for num1 in nums:
                for num2 in range(num1+1, s+1):
                    formula.append([-index(row, col, num1), -index(row, col, num2)])

            # All neighbors of a cell should not have the same numer
            cell_neighbors = neighbors(row, col, k)
            for num in nums:
                for sub_row, sub_col in cell_neighbors:
                    formula.append([-index(row, col, num), -index(sub_row, sub_col, num)])

            # Fill in existing values
            num = sudoku[row][col]
            if num != 0:
                formula.append([index(row, col, num)])            

    solver = MinisatGH()
    solver.append_formula(formula)
    answer = solver.solve()

    # Extract solution by looping through all cells and numbers
    solution = None
    if answer == True:
        model = solver.get_model()
        solution = [[0]*s for _ in rows]
        for row in rows:
            for col in cols:
                for num in nums:
                    if index(row, col, num) in model:
                        solution[row][col] = num
    
    return solution


def solve_sudoku_CSP(sudoku:list, k:int):
    '''
    Solver that uses CSP encoding
    The implementation is done similarly to ILP to improve performance on k >= 5
    '''
    s = k**2
    nums = list(range(1, s+1))
    rows = list(range(s))
    cols = list(range(s))

    model = cp_model.CpModel()

    # Every cell has a number between 1 ans s
    variables = [[[model.NewBoolVar(f"r{row}c{col}n{num}") for num in nums] for col in cols] for row in rows]

    for num in nums:
        # Numbers should not be the same as in the same row or column
        for col in cols:
            model.Add(sum(variables[row][col][num-1] for row in rows) == 1)
        for row in rows:
            model.Add(sum(variables[row][col][num-1] for col in cols) == 1)
    
        # Numbers should not be the same in the same box
        for box_row in range(k):
            for box_col in range(k):
                model.Add(sum(
                    variables[k*box_row + sub_row][k*box_col + sub_col][num-1]
                    for sub_row in range(k) for sub_col in range(k)
                ) == 1)

    for row in rows:
        for col in cols:
            # All cells should contain only 1 number
            model.Add(sum(variables[row][col][num-1] for num in nums) == 1)

            # Fill in existing numbers
            num = sudoku[row][col]
            if num != 0:
                model.Add(variables[row][col][num-1] == 1)

    solver = cp_model.CpSolver()
    answer = solver.Solve(model)

    # Extract solution by looping through all cells and numbers
    solution = None
    if answer == cp_model.FEASIBLE:
        solution = [[0]*s for _ in rows]
        for row in rows:
            for col in cols:
                for num in nums:
                    if solver.Value(variables[row][col][num-1]) == 1:
                        solution[row][col] = num

    return solution


def solve_sudoku_ASP(sudoku:list, k:int):
    '''
    Solver that uses ASP encoding
    '''
    s = k**2
    rows = list(range(s))
    cols = list(range(s))

    asp_code = f"""
    #const k={k}.
    #const s={s}.
    """

    # Define sudoku sizeand number range
    asp_code += """
    num(1..s).
    row(0..s-1).
    col(0..s-1).
    """
    
    # All cells should contain 1 number
    asp_code += """
    1 { cell(R, C, N) : num(N) } 1 :- row(R), col(C).
    """

    # Numbers should be different in rows and columns
    asp_code += """
    :- cell(RA, C, N), cell(RB, C, N), RA != RB.
    :- cell(R, CA, N), cell(R, CB, N), CA != CB.
    """

    # Numbers should be different in boxes
    asp_code += """
    :- cell(RA, CA, N), cell(RB, CB, N), RA/k == RB/k, CA/k == CB/k, RA != RB, CA != CB.
    """

    # Fill in existing values
    for row in rows:
        for col in cols:
            num = sudoku[row][col]
            if num != 0:
                asp_code += f"""
                cell({row}, {col}, {num}).
                """

    # Only output sudoku cells
    asp_code += """
    #show cell/3.
    """

    control = clingo.Control()
    control.add("base", [], asp_code)
    control.ground([("base", [])])

    control.configuration.solve.models = 1

    # Extract solution from model
    solution = None
    with control.solve(yield_=True) as handle:
        for model in handle:
            solution = [[0]*s for _ in range(s)]
            for cell in model.symbols(shown=True):
                row, col, num = [var.number for var in cell.arguments]
                solution[row][col] = num

    return solution


def solve_sudoku_ILP(sudoku:list, k:int):
    '''
    Solver that uses ILP encoding
    '''
    s = k**2
    nums = list(range(s))
    rows = list(range(s))
    cols = list(range(s))

    model = gurobipy.Model()

    cell = model.addVars(s, s, s, vtype=gurobipy.GRB.BINARY, name="cell")

    # All cells should contain 1 number
    for row in rows:
        for col in cols:
            model.addConstr(gurobipy.quicksum(cell[(row, col, num)] for num in nums) == 1)
    
    # All columns should have different numbers
    for row in rows:
        for num in nums:
            model.addConstr(gurobipy.quicksum(cell[(row, col, num)] for col in cols) == 1)

    # All rows should have different numbers
    for col in cols:
        for num in nums:
            model.addConstr(gurobipy.quicksum(cell[(row, col, num)] for row in rows) == 1)

    # All boxes should have different values
    for num in nums:
        for box_row in range(k):
            for box_col in range(k):
                model.addConstr(
                    gurobipy.quicksum(
                        cell[(k*box_row + sub_row, k*box_col + sub_col, num)]
                        for sub_row in range(k) for sub_col in range(k)
                    ) == 1
                )

    # Fill in existing values
    for row in rows:
        for col in cols:
            num = sudoku[row][col]
            if num != 0:
                model.addConstr(cell[(row, col, num-1)] == 1)

    model.optimize()

    # Extract solution by looping through all cells and numbers
    solution = None
    if model.status == gurobipy.GRB.OPTIMAL:
        solution = [[0]*s for _ in rows]
        for row in rows:
            for col in cols:
                for num in nums:
                    if cell[(row, col, num)].x == 1:
                        solution[row][col] = num + 1

    return solution
