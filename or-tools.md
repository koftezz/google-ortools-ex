# Google OR-Tools

[https://google.github.io/or-tools/](https://google.github.io/or-tools/)

[https://github.com/koftezz/google-ortools-ex](https://github.com/koftezz/google-ortools-ex)

OR-Tools is an open source software for optimization problems in vehicle routing, flows, integer and linear programming, and constraint programming. We can call commercial solvers such as Gurobi or CPLEX, or open-source solvers such as SCIP, GLPK, or Google's GLOP and award-winning CP-SAT.

There are 5 modules.

- [Algorithms module](https://google.github.io/or-tools/python/ortools/algorithms/pywrapknapsack_solver.html)
- [Graph module](https://google.github.io/or-tools/python/ortools/graph/pywrapgraph.html)
- [Linear Solver module](https://google.github.io/or-tools/python/ortools/linear_solver/pywraplp.html)
- [CP-SAT module](https://google.github.io/or-tools/python/ortools/sat/python/cp_model.html)
- [Routing module](https://google.github.io/or-tools/python/ortools/constraint_solver/pywrapcp.html)
- [Domain module](https://google.github.io/or-tools/python/ortools/util/sorted_interval_list.html)

This is a T[wo-index vehicle flow formulation](https://arxiv.org/pdf/1606.01935.pdf) with [Miller-Tucker-Zemlin subtour elimination constraint.](https://how-to.aimms.com/Articles/332/332-Miller-Tucker-Zemlin-formulation.html)

![Google%20OR-Tools%2041fee5f704b949a6972b7f378b6168af/Untitled.png](Google%20OR-Tools%2041fee5f704b949a6972b7f378b6168af/Untitled.png)

$c_{i, j}$: cost matrix

$q_{i}$: demand

$x_{i, j}$: binary decision variable $x_{i, j}$ that assumes value 1 if and only if there is a route that goes from customer i to j directly.

$y_{i}$: continuous decision variable corresponding to the cumulated demand on the route that visits ode

- Consider a set of customers represented by C = {1, . . . , n}, such that a positive demand
is associated to each customer i ∈ C.
- To service these customers, we have to design routes for a fleet with K vehicles available in a single depot.
- Each route must start at the depot, visit a subset of customers and then return to the depot.
- All customers must be visited exactly once.
- Each vehicle has a maximum capacity Q, which limits the number of customers it can
visit before returning to the depot.
- We assume a homogeneous fleet of vehicles.
- Define set of nodes associated to customers in C and to the depot nodes 0 and n + 1. We use two nodes to represent the same single depot and impose that all routes must start on 0 and return to n + 1.
- The cost of crossing an arc (i, j) ∈ E is denoted by cij . Each node has a demand qi, such that qi > 0 for each i ∈ C and q0 = qn+1 = 0.
- The objective of the problem is to determine a set of minimal cost routes that satisfies all the requirements defined above.
- Constraints (2.2) ensure that all customers are visited exactly once.
- Constraints (2.3) guarantee the correct flow of vehicles through the arcs, by stating that if a vehicle arrives to a node h ∈ N , then it must depart from this node.
- Constraint (2.4) limits the maximum number of routes to K, the number of vehicles.
- Constraints (2.5) and (2.6) ensure together that the vehicle capacity is not exceeded.
- The objective function is defined by (2.1) and imposes that the total travel cost of the routes is minimized.
- Constraints (2.5) also avoid subtours in the solution, i.e. cycling routes that do not pass
through the depot. Different types of constraints are proposed in the literature to impose
vehicle capacities and/or avoid subtours [17]. The advantage of using (2.5) and (2.6) is that
the model has a polynomial number of constraints in terms of the number of customers.
However, the lower bound provided by the linear relaxation of this model is known to be
weak in relation to other models