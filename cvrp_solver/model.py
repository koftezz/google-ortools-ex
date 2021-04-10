from ortools.linear_solver import pywraplp

# from cvrp_solver.constraints import Constraints
# from cvrp_solver.optimization_params import OptimizationParams
# from cvrp_solver.problem_params import ProblemParams
# from cvrp_solver.objective_function import ObjectiveFunction
# from cvrp_solver.data_generation import DataGeneration
from constraints import Constraints
# from variables import Variables
from cvrp_solver.objective_function import ObjectiveFunction


class Model(object):

    def __init__(self,
                 # data,
                 # optimization_params,
                 # problem_params
                 # vehicles,
                 # customers,
                 # capacity
                 ):
        # self.vehicles = vehicles
        # self.customers = customers
        # self.capacity = capacity
        # self.data = data
        # self.problemParams = selfproblem_params
        # self.optimizationParams = optimization_params
        self.x = dict()
        self.y = dict()


    # self.problemParams = ProblemParams(self.data)
    # self.optimizationParams = OptimizationParams()
        self.model = Model.create_model(self)
        print(self.model)
        self.model = Model.generate_variables(self)
        print(self.model.NumVariables())
        Model.generate_constraints(self)
        print(self.model.NumConstraints())
        Model.objective_function(self)

        # model = Constraints(self.model, self.problemParams)
        # print(self.model.NumVariables())
        # model = ObjectiveFunction(model, self.problemParams)
        #
        if self.optimization_params.enable_output:
            self.model.EnableOutput()
        if self.optimization_params.time_limit:
            self.model.SetTimeLimit(self.optimization_params.time_limit)
        self.model.Solve()
        Model.print_solution(self)

    def generate_variables(self):
        V = self.problem_params['V']
        print(V)
        for i in V:
            for j in V:
                self.x[i, j] = self.model.BoolVar(name='x[{0}][{1}]'.format(i,
                                                                            j))

        for j in V:
            self.y[j] = self.model.NumVar(lb=0, ub=self.model.infinity(),
                                          name='y[{0}]'.format(j))
        return self.model
        # print(self.model)
        # self.model = Constraints(self.model, self.problemParams)
        # self.model = ObjectiveFunction(self.model, self.optimizationParams)

    # @staticmethod
    def create_model(self):
        # model = pywraplp.Solver.CreateSolver('SCIP')
        if self.optimization_params.solver == "MIP":
            model = pywraplp.Solver('CVRP',
                            pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        elif self.optimization_params.solver == "SCIP":
            model = pywraplp.Solver.CreateSolver(self.optimization_params.solver)
        else:
            print("undefined solver")
            pass
        return model

    def generate_constraints(self):
        V = self.problem_params['V']
        model = self.model
        for i in V[1:-1]:
            model.Add(model.Sum(self.x[i, j] for j in V[1:]
                                if j != i) == 1)

        for h in V[1:-1]:
            model.Add((model.Sum(self.x[i, h] for i in V[:-1]
                                 if h != i)) == (
                          model.Sum(self.x[h, j] for j in V[1:]
                                    if j != h)))

        model.Add(model.Sum(self.x[0, j] for j in V[1:-1])
                  <= self.problem_params['number_of_vehicles'])

        q = self.problem_params['q']
        Q = self.problem_params['Q']
        for i in V:
            for j in V:
                model.Add(self.y[j] >= self.y[i] + q[j] *
                          self.x[i, j] -
                          Q * (1 - self.x[i, j]))

        for i in V:
            model.Add(q[i] <= self.y[i])
            model.Add(self.y[i] <= Q)

    def objective_function(self):
        c = self.problem_params['c']
        V = self.problem_params['V']
        self.model.Minimize(self.model.Sum(self.x[i, j]
                                           * c[i, j] for i in V for j in V))

    def print_solution(self):
        V = self.problem_params['V']
        x = self.x
        K = self.problem_params['number_of_vehicles']

        num_of_vehicles = 0
        for j in V:
            if x[0, j].SolutionValue() == 1:
                num_of_vehicles += 1
        print(
            "Total {0} out of {1}  vehicles used.".format(num_of_vehicles, K))

        res = dict()
        for i in V:
            for j in V:
                if x[i, j].SolutionValue() == 1:
                    # print("x{0}_{1}".format(i, j), x[i, j].SolutionValue())
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
                        if j == len(V) - 1:
                            a = False
                    else:
                        j += 1
                        if j == len(V) - 1:
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
            output += "Node: {0}".format(len(V) - 1)
            print(output)