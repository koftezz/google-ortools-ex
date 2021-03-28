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


def get_max_available_position(number):
    return {
        0: 1,
        1: 1,
        2: 2,
        3: 2,
        4: 2,
    }[number]


# players = data

def construct_model(players, salaryCap):
    totalTeams = players['Team'].unique()
    players_dict = [[], [], [], [], []]
    # players_dict = players
    for i, row in players.iterrows():
        players_dict[get_position_number(row['Subposition'])].append(
            [row['Name'], float(row['Value']), float(row['Salary']),
             int(row['Team'])]
        )

    solver = pywraplp.Solver('NBA_Opt',
                             pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

    position_dict = dict()
    for i in range(5):
        position_dict[i] = range(len(players_dict[i]))

    # rangeC = range(len(players_dict[0]))
    # rangePG = range(len(players[1]))
    # rangePF = range(len(players[2]))
    # rangeSG = range(len(players[3]))
    # rangeSF = range(len(players[4]))

    get_position = dict()
    for pos in position_dict.keys():
        get_position[pos] = [
            solver.IntVar(0, 1, 'get_position_{0}[{1}]'.format(
                get_position_name(pos), j)) for j in position_dict[pos]]

    # takeC = [solver.IntVar(0, 1, 'takeC[%i]' % j) for j in rangeC]
    # takePG = [solver.IntVar(0, 1, 'takePG[%i]' % j) for j in rangePG]
    # takePF = [solver.IntVar(0, 1, 'takePF[%i]' % j) for j in rangePF]
    # takeSG = [solver.IntVar(0, 1, 'takeSG[%i]' % j) for j in rangeSG]
    # takeSF = [solver.IntVar(0, 1, 'takeSF[%i]' % j) for j in rangeSF]

    # namePG = []
    # namePF = []
    # nameSG = []
    # nameSF = []

    team_dict = {}
    # max
    for pos in position_dict.keys():
        name_list = []
        for teamNumber in range(0, 58):
            name_list.insert(teamNumber, solver.Sum(
                [(players_dict[pos][j][3] == teamNumber + 1)
                 * get_position[pos][j]
                 for j in position_dict[pos]]))
        team_dict[pos] = name_list

        # team_dict[pos] = solver.Sum([(players_dict[pos][j][3] == teamNumber)
        #                              * get_position[pos][j]
        #                              for j in position_dict[pos]])

    value_dict = {}
    # max
    # for teamNumber in totalTeams:
    for pos in position_dict.keys():
        value_dict[pos] = solver.Sum(
            [players_dict[pos][j][1] * get_position[pos][j]
             for j in position_dict[pos]])

    salary_dict = {}
    # max
    for pos in position_dict.keys():
        salary_dict[pos] = solver.Sum(
            [players_dict[pos][j][2] * get_position[pos][j]
             for j in position_dict[pos]])
    #
    # salaryC = solver.Sum([players[0][i][2] * takeC[i] for i in rangeC])
    # salaryPG = solver.Sum([players[1][i][2] * takePG[i] for i in rangePG])
    # salaryPF = solver.Sum([players[2][i][2] * takePF[i] for i in rangePF])
    # salarySG = solver.Sum([players[3][i][2] * takeSG[i] for i in rangeSG])
    # salarySF = solver.Sum([players[4][i][2] * takeSF[i] for i in rangeSF])

    # Budget Constraint
    solver.Add(sum(salary_dict.values()) <= salaryCap)

    # Max Available Position Constraint
    for pos in position_dict.keys():
        solver.Add(solver.Sum(get_position[pos][j]
                              for j in position_dict[pos]) ==
                   get_max_available_position(pos))

    # solver.Add(solver.Sum(takeC[i] for i in rangeC) == 1)
    # solver.Add(solver.Sum(takePG[i] for i in rangePG) == 1)
    # solver.Add(solver.Sum(takePF[i] for i in rangePF) == 2)
    # solver.Add(solver.Sum(takeSG[i] for i in rangeSG) == 2)
    # solver.Add(solver.Sum(takeSF[i] for i in rangeSF) == 2)
    # team_dict
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
