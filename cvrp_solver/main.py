from model import Model
from optimization_params import OptimizationParams
from problem_params import ProblemParams
from data_generation import DataGeneration


class Solve(object):
    def __init__(self):
        self.optimization_params = OptimizationParams()
        self.data = DataGeneration.create_random_data(vehicles=5,
                                                      customers=20,
                                                      capacity=15)

        ProblemParams.__init__(self)
        Model.__init__(self)


if __name__ == "__main__":
    Solve()
