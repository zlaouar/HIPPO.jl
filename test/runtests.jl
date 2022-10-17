using HIPPO
using POMDPs
using BasicPOMCP
using POMDPTools

m = TargetSearchPOMDP()

transition(m, TSState([1,1], [10,10]), :left)

solver = POMCPSolver()
planner = solve(solver, m)

ds = DisplaySimulator()

simulate(ds, m, planner)