from ortools.linear_solver import pywraplp
from gurobipy import *
import numpy as np


def create_data(customers=20,
                vehicles=5,
                capacity=20):
    rnd = np.random
    rnd.seed(0)
    xc = rnd.rand(customers + 1) * 200
    yc = rnd.rand(customers + 1) * 100
    xc = np.append(xc, xc[0])
    yc = np.append(yc, yc[0])
    V = [i for i in range(customers + 1)]
    V.append(customers + 1)
    Q = capacity
    q = {i: rnd.randint(1, 3) for i in V}
    q[0] = 0
    q[len(V)] = 0

    c = {(i, j): np.hypot(xc[i] - xc[j], yc[i] - yc[j])
         for i in V for j in V}

    data = dict()
    data['c'] = c
    data['V'] = V
    data['q'] = q
    data['Q'] = Q
    data['K'] = vehicles
    return data


def create_model(data, print_log=True, time_limit=None):

    s = pywraplp.Solver.CreateSolver('SCIP')
    V = data['V']
    K = data['K']
    Q = data['Q']
    q = data['q']
    c = data['c']

    x = dict()
    for i in V:
        for j in V:
            x[(i, j)] = s.BoolVar(name='x[{0}][{1}]'.format(i, j))

    y = dict()
    for j in V:
        y[j] = s.NumVar(lb=0, ub=s.infinity(), name='y[{0}]'.format(j))

    s.Minimize(s.Sum(x[i, j] * c[i, j] for i in V for j in V))

    # const 1
    for i in V[1:-1]:
        s.Add(s.Sum(x[i, j] for j in V[1:] if j != i) == 1)
    # const 2
    for h in V[1:-1]:
        s.Add((s.Sum(x[i, h] for i in V[:-1]
                     if h != i)) == (s.Sum(x[h, j] for j in V[1:]
                                           if j != h)))
    s.Add(s.Sum(x[0, j] for j in V[1:-1]) <= K)

    for i in V:
        for j in V:
            s.Add(y[j] >= y[i] + q[j] * x[i, j] - Q * (
                    1 - x[i, j]))  # q[j]* x[i, j]

    for i in V:
        s.Add(q[i] <= y[i])
        s.Add(y[i] <= Q)

    print("Number of variables: ", s.NumVariables())
    print("Number of constraints: ", s.NumConstraints())
    print_log = False
    if print_log:
        s.EnableOutput()

    if time_limit:
        s.SetTimeLimit(time_limit)
    status = s.Solve()
    model = dict()
    model['data'] = data
    model['x'] = x
    model['y'] = y

    if status == pywraplp.Solver.OPTIMAL:
        print('Solution:')
        print('Objective value =', s.Objective().Value())
    else:
        print('The problem does not have an optimal solution.')
    print_solution(model)
    print('\nAdvanced usage:')
    print('Problem solved in %f milliseconds' % s.wall_time())
    print('Problem solved in %d iterations' % s.iterations())
    print('Problem solved in %d branch-and-bound nodes' % s.nodes())
    print(model)
    return model


def print_solution(model):
    V = model['data']['V']
    x = model['x']
    K = model['data']['K']

    num_of_vehicles = 0
    for j in V:
        if x[0, j].SolutionValue() == 1:
            num_of_vehicles += 1
    print("Total {0} out of {1}  vehicles used.".format(num_of_vehicles, K))


    res = dict()
    for i in V:
        for j in V:
            if x[i, j].SolutionValue() == 1:
                print("x{0}_{1}".format(i, j), x[i, j].SolutionValue())
                res[i, j] = 1

    vehicle = 0
    while vehicle < num_of_vehicles:
        route = []
        i = 0
        route.append(i)
        j = 0
        a = True
        while a:
            try:
                if res[i, j] == 1:
                    route.append(j)
                    del res[i, j]
                    i = j
                    j = 0
                    if j == len(V)-1:
                        a = False
                else:
                    j += 1
                    if j == len(V)-1:
                        a = False
            except:
                j += 1
                if j == len(V) - 1:
                    a = False
        vehicle += 1
        output = "Route {0} for Vehicle {1}\n".format(vehicle, vehicle)
        # route.append(len(V)-1)
        for i in route:
            output += "Node: {0} -> ".format(i)
        output += "Node: {0}".format(len(V)-1)
        print(output)


def create_gurobi_model(data, time_limit=None):
    V = data['V']
    K = data['K']
    Q = data['Q']
    q = data['q']
    c = data['c']

    m = Model("CVRP")

    x = dict()
    for i in V:
        for j in V:
            x[(i, j)] = m.addVar(vtype=GRB.BINARY,
                                 name='x[{0}][{1}]'.format(i, j))

    y = dict()
    for j in V:
        y[j] = m.addVar(vtype=GRB.CONTINUOUS, lb=0, name='y[{0}]'.format(j))

    obj = (quicksum(x[i, j] * c[i, j] for i in V for j in V))
    m.setObjective(obj, GRB.MINIMIZE)

    # const 1
    for i in V[1:-1]:
        m.addConstr(quicksum(x[i, j] for j in V[1:] if j != i) == 1)
    # const 2
    for h in V[1:-1]:
        m.addConstr((quicksum(x[i, h] for i in V[:-1]
                              if h != i)) == (quicksum(x[h, j] for j in V[1:]
                                                       if j != h)))

    m.addConstr(quicksum(x[0, j] for j in V[1:-1]) <= K)

    for i in V:
        for j in V:
            m.addConstr(y[j] >= y[i] + q[j] * x[i, j] - Q * (
                    1 - x[i, j]))

    for i in V:
        m.addConstr(q[i] <= y[i])
        m.addConstr(y[i] <= Q)

    m.update()

    # m.setParam("MIPGap", 0.80)
    if time_limit:
        m.setParam("TimeLimit", time_limit)
    m.optimize()


if __name__ == "__main__":

    data = create_data(vehicles=5, customers=5, capacity=6)
    model = create_model(data, time_limit=10000)# milliseconds
    create_gurobi_model(data, time_limit=100) #seconds