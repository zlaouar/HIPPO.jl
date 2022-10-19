using HIPPO
using POMDPs
using QMDP
using BasicPOMCP
using POMDPTools

m = TargetSearchPOMDP()

transition(m, TSState([1,1], [10,10]), :left)

#solver = POMCPSolver()
solver = QMDPSolver(max_iterations=20,
                    belres=1e-3,
                    verbose=true
                   ) 
planner = solve(solver, m)

ds = DisplaySimulator()

simulate(ds, m, planner)