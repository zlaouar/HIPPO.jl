using HIPPO
using POMDPs
using QMDP
using BasicPOMCP
using POMDPTools
using DiscreteValueIteration
using Profile
using ParticleFilters
using D3Trees


sleep_until(t) = sleep(max(t-time(), 0.0))

function custom_sim(msolve::TargetSearchPOMDP, msim::TargetSearchPOMDP, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 2
    dt = 1/max_fps
    d = 1.0
    sim_states = TSState[]
    while !isterminal(msim, s)
        tm = time()
        a = action(planner, b)
        s, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        display(render(msim, (sp=s, bp=b)))
        sleep_until(tm += dt)
        iter += 1
        println(iter)
        if iter > 100
            roi_states = [[1,9],[1,10],[1,8]]
            probs = [0.8,0.8,0.8]
            roi_points = Dict(roi_states .=> probs)
            msolve.rois = roi_points
            planner = solve(solver,msolve)
        end
        push!(sim_states, s)
    end
    return s, r_total, sim_states
end

sinit = TSState([4,1],[2,5])#rand(initialstate(msim))
roi_states = [[5,5]]
probs = [0.8]
roi_points = Dict(roi_states .=> probs)
#msolve= TargetSearchPOMDP(roi_points=roi_points)
msolve= TargetSearchPOMDP(sinit=sinit, size=(5,5), roi_points=roi_points)
mdp_solver = ValueIterationSolver() # creates the solver
mdp_policy = solve(mdp_solver, UnderlyingMDP(msolve))



#estimate_value=FORollout(mdp_policy)
solver = POMCPSolver(estimate_value=FORollout(mdp_policy), tree_queries=10000, max_time=0.2, c=3)
#solver = POMCPSolver(tree_queries=10000, max_time=0.2, c=3)
#solver = QMDPSolver(max_iterations=20,
#                    belres=1e-3,
#                    verbose=true
#                   ) 

planner = solve(solver,msolve)

ds = DisplaySimulator()
hr = HistoryRecorder()
msim = TargetSearchPOMDP(sinit=sinit, size=(5,5))

b0 = initialstate(msolve)
up = DiscreteUpdater(msolve)
b = initialize_belief(up, b0)

N = 1000
particle_up = BootstrapFilter(msolve, N)
particle_b = initialize_belief(particle_up, b0)



#a, info = action_info(planner, Deterministic(TSState([13,14],[1,1])), tree_in_info=true)
#inchrome(D3Tree(info[:tree], init_expand=3))

s,r_total,sim_states  = custom_sim(msolve, msim, planner, particle_up, particle_b, sinit)

#r_total
#h = simulate(ds, msim, planner)
#h = simulate(ds, msim, planner, particle_up, initialstate(msim), sinit)


#@profview action(planner, initialstate(m))
#= for i in 1:length(h)
    display(render(m, (sp=h[i].s, bp=h[i].b)))
    sleep(0.1)
end =#
