from planning import PlanningProblem, Action, Expr, expr
import planning

import clingo

def translate(planning_problem:PlanningProblem, trans:dict):

    def recurse(exp:Expr):
        try:
            if isinstance(exp, Action):
                exp.name = trans[exp.name]
            else:
                exp.op = trans[exp.op]
        except KeyError:
            pass
        
        for arg in exp.args:
            recurse(arg)


    for initial in planning_problem.initial:
        recurse(initial)
    
    for action in planning_problem.actions:
        recurse(action)
        for precond in action.precond:
            recurse(precond)
        for effect in action.effect:
            recurse(effect)
    
    for goal in planning_problem.goals:
        recurse(goal)


def build_translation(planning_problem:PlanningProblem):
    trans = {"~": "~"}

    def recurse(exp):
        if isinstance(exp, Action):
            name = exp.name
        else:
            name = exp.op
        
        if name not in trans:
            if name[0].isupper():
                trans[name] = "atom_" + name
            else:
                trans[name] = "VAR_" + name
        
        for arg in exp.args:
            recurse(arg)


    for initial in planning_problem.initial:
        recurse(initial)
    
    for action in planning_problem.actions:
        recurse(action)
        for precond in action.precond:
            recurse(precond)
        for effect in action.effect:
            recurse(effect)
    
    for goal in planning_problem.goals:
        recurse(goal)

    return trans


def solve_planning_problem_using_ASP(planning_problem:PlanningProblem, t_max:int):
    """
    If there is a plan of length at most t_max that achieves the goals of a given planning problem,
    starting from the initial state in the planning problem, returns such a plan of minimal length.
    If no such plan exists of length at most t_max, returns None.

    Finding a shortest plan is done by encoding the problem into ASP, calling clingo to find an
    optimized answer set of the constructed logic program, and extracting a shortest plan from this
    optimized answer set.

    NOTE: still needs to be implemented. Currently returns None for every input.

    Parameters:
        planning_problem (PlanningProblem): Planning problem for which a shortest plan is to be found.
        t_max (int): The upper bound on the length of plans to consider.

    Returns:
        (list(Expr)): A list of expressions (each of which specifies a ground action) that composes
        a shortest plan for planning_problem (if some plan of length at most t_max exists),
        and None otherwise.
    """

    # Create translation and inverse translation to support clingo's capital use case
    trans = build_translation(planning_problem)
    translate(planning_problem, trans)

    # Define timesteps
    asp_code = f"#const t_max={t_max}.\n"
    asp_code += "time(0..t_max).\n"

    # Add initial states
    for initial in planning_problem.initial:
        asp_code += f"state(0, {initial}).\n"

    for action in planning_problem.actions:
        # List actions that are available regarding preconditions
        preconds = ", ".join([   
            f"not state(T-1, {precond.args})"
            if precond.op == "~" else
            f"state(T-1, {precond})"
            for precond in action.precond
        ] + ["time(T)"])
        asp_code += f"available(T, {action}) :- {preconds}.\n"

        # Add action effects if one is chosen
        effects = ", ".join(f"state(T, {effect})" for effect in action.effect if effect.op != "~")
        #[f"not state(T, {effect.args})" for effect in action.effect if effect.op == "~"]
        asp_code += f"{effects} :- chosen(T, {action}), time(T).\n"
    
    # Add goals
    for goal in planning_problem.goals:
        asp_code += f"goal({goal}).\n"

    # Inherit previous states
    # TODO Except if negated by effects
    asp_code += "state(T, E) :- state(T-1, E), time(T).\n"

    # One action can be taken at a timestep if still busy
    asp_code += "1 { chosen(T, A) : available(T, A) } 1 :- time(T), busy(T-1).\n"

    # Require goals to be met before t_max
    asp_code += ":- not state(t_max, G), goal(G).\n"

    # Define when the problem is solved
    asp_code += "busy(T) :- not state(T, G), goal(G), time(T).\n"
    asp_code += "1 { done(T) : not busy(T), time(T) } 1.\n"

    # Configure optimization and output
    asp_code += "#minimize { T, done(T) : done(T) }.\n"
    asp_code += "#show chosen/2.\n"

    #print(asp_code)

    # Configure clingo to find 1 optimal model
    control = clingo.Control()
    control.add("base", [], asp_code)
    control.ground([("base", [])])
    control.configuration.solve.models = 1
    control.configuration.solve.opt_mode = "optN"

    # Retrieve the solution as a plan
    plan = None
    with control.solve(yield_=True) as handle:
        for model in handle:
            if model.optimality_proven == True:
                #for item in model.symbols(shown=True):
                #    print(item)
                plan = [expr(str(action).replace("atom_", "")) for _, action in sorted(action.arguments for action in model.symbols(shown=True))]

    return plan
