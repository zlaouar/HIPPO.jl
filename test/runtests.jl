using HIPPO
using POMDPs
using QMDP
using BasicPOMCP
using POMDPTools

roi_states = [[7,9],[7,10],[7,8]]
probs = [0.3,0.3,0.3]
values = [10,10,10]
roi_points = Dict(roi_states .=> probs)
m = TargetSearchPOMDP(roi_points=roi_points)

transition(m, TSState([1,1], [10,10]), :left)

solver = POMCPSolver()
#solver = QMDPSolver(max_iterations=20,
#                    belres=1e-3,
#                    verbose=true
#                   ) 

planner = solve(solver, m)

ds = DisplaySimulator()
hr = HistoryRecorder()
m1 = TargetSearchPOMDP()

simulate(ds, m1, planner)

