using HIPPO
using POMDPs
using QMDP
using BasicPOMCP
using POMDPTools
using DiscreteValueIteration
using Profile


sleep_until(t) = sleep(max(t-time(), 0.0))

function customRollout(m::TargetSearchPOMDP, h::BeliefNode, steps)

end

roi_states = [[7,9],[7,10],[7,8]]
probs = [0.8,0.8,0.8]
values = [10,10,10]
roi_points = Dict(roi_states .=> probs)
m = TargetSearchPOMDP(roi_points=roi_points)
mdp_solver = ValueIterationSolver() # creates the solver
mdp_policy = solve(mdp_solver, UnderlyingMDP(m))
#m = TargetSearchPOMDP(roi_points=roi_points)

transition(m, TSState([1,1], [10,10]), :left)

#estimate_value=FORollout(mdp_policy)
solver = POMCPSolver(estimate_value=FORollout(mdp_policy), tree_queries=10000, max_time=0.2)
#solver = QMDPSolver(max_iterations=20,
#                    belres=1e-3,
#                    verbose=true
#                   ) 

planner = solve(solver, m)

ds = DisplaySimulator()
hr = HistoryRecorder()
m1 = TargetSearchPOMDP()

b0 = initialstate(m1)
up = DiscreteUpdater(m1)
b = initialize_belief(up, b0)

r_total = 0.0
d = 1.0
sinit = TSState([10,1],[2,7])#rand(initialstate(m1))
s = sinit
o = Nothing
iter = 0
max_fps = 10
dt = 1/max_fps

while !isterminal(m1, s)
    tm = time()
    a = action(planner, b)
    s, o, r = @gen(:sp,:o,:r)(m1, s, a)
    r_total += d*r
    d *= discount(m1)
    b = update(up, b, a, o)
    display(render(m1, (sp=s, bp=b)))
    sleep_until(tm += dt)
    iter += 1
    println(iter)
    if iter > 100
        roi_states = [[1,9],[1,10],[1,8]]
        probs = [0.3,0.3,0.3]
        roi_points = Dict(roi_states .=> probs)
        m.rois = roi_points
        #m = TargetSearchPOMDP(roi_points=roi_points)
        planner = solve(solver, m)
    end
end
println(s)
println(iter)
r_total
h = simulate(ds, m1, planner, up, initialstate(m1), sinit)


#@profview action(planner, initialstate(m))
for i in 1:length(h)
    display(render(m, (sp=h[i].s,)))
    sleep(0.1)
end
