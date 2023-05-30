using HIPPO
using POMDPs
using BasicPOMCP
using POMDPTools
using DiscreteValueIteration
using Profile
using ParticleFilters
using D3Trees
using JSON


#mapsize = (13,16)
#sinit = TSState([10,1],[13,16],trues(prod(mapsize)))#rand(initialstate(msim))
mapsize = (4,4)
maxbatt = 10
sinit = TSStateBattery([1,1],mapsize,maxbatt) #rand(initialstate(msim))



msolve = TSPOMDPBattery(sinit, size=mapsize, maxbatt=maxbatt)
#mdp_solver = ValueIterationSolver() # creates the solver
#mdp_policy = solve(mdp_solver, UnderlyingMDP(msolve))

#p = FunctionPolicy(FixedPolicy())
#mdprollout = FORollout(TargetSearchMDPPolicy(mdp_policy))
#funcrollout = FORollout(p)
#mdprollout = FORollout(mdp_policy) # change MDP reward mat to pompdp reward mat
solver = POMCPSolver(tree_queries=10_000, max_time=0.2, c=20000)
#solver = POMCPSolver(tree_queries=10000, max_time=0.2, c=5)
planner = solve(solver,msolve)

ds = DisplaySimulator()
hr = HistoryRecorder()
msim = TSPOMDPBattery(sinit, size=mapsize, maxbatt=maxbatt)

b0 = initialstate(msolve)

N = 1000
particle_up = BootstrapFilter(msolve, N)
particle_b = initialize_belief(particle_up, b0)


#a, info = action_info(planner, Deterministic(TSState([13,14],[1,1])), tree_in_info=true)
#inchrome(D3Tree(info[:tree], init_expand=3))

r_total, sim_states, belframes = customsim(msolve, msim, planner, particle_up, particle_b, sinit)


display("Simulation Ended")


#r_total
#h = simulate(ds, msim, planner)
#h = simulate(ds, msim, planner, particle_up, initialstate(msim), sinit)


#@profview action(planner, initialstate(m))
#= for i in 1:length(h)
    display(render(m, (sp=h[i].s, bp=h[i].b)))
    sleep(0.1)
end =#

#write("reward.gif", rewardframes)
