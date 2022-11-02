using HIPPO
using POMDPs
using QMDP
using BasicPOMCP
using POMDPTools


sleep_until(t) = sleep(max(t-time(), 0.0))


roi_states = [[7,9],[7,10],[7,8]]
probs = [0.3,0.3,0.3]
values = [10,10,10]
roi_points = Dict(roi_states .=> probs)
m = TargetSearchPOMDP(roi_points=roi_points)

transition(m, TSState([1,1], [10,10]), :left)

solver = POMCPSolver(max_depth=100, c=1.5)
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
s = TSState([10,1],[2,7])#rand(initialstate(m1))
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
    display(render(m, (sp=s, bp=b)))
    sleep_until(tm += dt)
    iter += 1
    println(iter)
    if iter > 100
        roi_states = [[1,9],[1,10],[1,8]]
        probs = [0.3,0.3,0.3]
        roi_points = Dict(roi_states .=> probs)
        m = TargetSearchPOMDP(roi_points=roi_points)
        planner = solve(solver, m)
    end
end
println(s)
println(iter)
r_total
#h = simulate(hr, m1, planner)

