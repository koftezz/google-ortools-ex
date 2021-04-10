import logging
import numpy as np


class DataGeneration:

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        # self.vehicles = vehicles
        # self.customers = customers
        # self.capacity = capacity
        # self.create_random_data()

    @staticmethod
    def create_random_data(
            vehicles,
            customers,
            capacity):
        data = dict()
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
        q[len(V)-1] = 0
        c = {(i, j): np.hypot(xc[i] - xc[j], yc[i] - yc[j]) for i in V
             for j in V}

        data['c'] = c
        data['V'] = V
        data['K'] = vehicles
        data['Q'] = Q
        data['q'] = q
        print(data)
        return data
