# NBA Lineup optimizer
# 06.04.2018
# by Back Bombers || Volkan Selim Cantürk ||Zeynel Batuhan Organ ||Eren Atsız
#
# Picks an ideal fantasy NBA team using a modified knapsack algorithm
#
# Usage: python nba-optimizer.py players.csv

import csv, sys
import pandas as pd
import os
from ortools.linear_solver import pywraplp


def get_position_number(name):
    return {
        'C': 0,
        'PG': 1,
        'PF': 2,
        'SG': 3,
        'SF': 4,
    }[name]


def get_position_name(number):
    return {
        0: 'C',
        1: 'PG',
        2: 'PF',
        3: 'SG',
        4: 'SF',
    }[number]


def get_available_position(number):
    return {
        0: 1,
        1: 1,
        2: 2,
        3: 2,
        4: 2,
    }[number]


def construct_model(players, salaryCap):
    players_dict = [[], [], [], [], []]
    for i, row in players.iterrows():
        players_dict[get_position_number(row['Subposition'])].append(
            [row['Name'], float(row['Value']), float(row['Salary']),
             int(row['Team'])]
        )

    solver = pywraplp.Solver('NBA_Opt',
                             pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    # get player ranges for position
    position_dict = dict()
    for i in range(5):
        position_dict[i] = range(len(players_dict[i]))

    # add decision variables for player
    get_position = dict()
    for pos in position_dict.keys():
        get_position[pos] = [
            solver.IntVar(lb=0, ub=1, name='get_position_{0}[{1}]'.format(
                get_position_name(pos), j)) for j in position_dict[pos]]

    team_dict = {}
    for pos in position_dict.keys():
        name_list = []
        for teamNumber in range(0, 58):
            name_list.insert(teamNumber, solver.Sum(
                [(players_dict[pos][j][3] == teamNumber + 1)
                 * get_position[pos][j]
                 for j in position_dict[pos]]))
        team_dict[pos] = name_list

    # fantasy point parameters
    value_dict = {}
    for pos in position_dict.keys():
        value_dict[pos] = solver.Sum(
            [players_dict[pos][j][1] * get_position[pos][j]
             for j in position_dict[pos]])

    # salary parameters
    salary_dict = {}
    for pos in position_dict.keys():
        salary_dict[pos] = solver.Sum(
            [players_dict[pos][j][2] * get_position[pos][j]
             for j in position_dict[pos]])

    # Budget Constraint
    solver.Add(sum(salary_dict.values()) <= salaryCap)

    # Available Position Constraint
    for pos in position_dict.keys():
        solver.Add(solver.Sum(get_position[pos][j]
                              for j in position_dict[pos]) ==
                   get_available_position(pos))

    # Max 4 players per team Constraint
    for i in range(0, 58):
        solver.Add(
            solver.Sum(team_dict[team][i] for team in team_dict.keys()) <= 1)

    solver.Maximize(solver.Sum(value_dict.values()))
    solver.Solve()
    assert solver.VerifySolution(1e-7, True)
    print('Solved in', solver.wall_time(), 'milliseconds!', "\n")

    salary = 0
    for pos in position_dict.keys():
        for i in position_dict[pos]:
            if get_position[pos][i].SolutionValue():
                salary += players_dict[pos][i][2]
                print(players_dict[pos][i][0],
                      '({0}): ${1}'.format(get_position_name(pos),
                                           players_dict[pos][i][2]),
                      '(' + str(players_dict[pos][i][1]) + ')')
    print("\n", 'Total: ${:,f}'.format(salary), '(' + str(
        solver.Objective().Value()) + ')')


if __name__ == "__main__":
    data = pd.read_csv(os.path.join(os.getcwd(), "data",
                                    "fantasy_points" + ".csv"))
    construct_model(players=data, salaryCap=50000)
