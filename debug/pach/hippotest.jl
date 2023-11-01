using HIPPO
using POMDPs
using POMDPTools
using BasicPOMCP
using DiscreteValueIteration
using ParticleFilters
using JLD2

db = load(joinpath(@__DIR__, "../data/db.jld2"), "db")
inputs1 = load(joinpath(@__DIR__, "../data/opdata1.jld2"), "inputs")
inputs2 = load(joinpath(@__DIR__, "../data/opdata2.jld2"), "inputs")
inputs3 = load(joinpath(@__DIR__, "../data/opdata3.jld2"), "inputs")
inputs4 = load(joinpath(@__DIR__, "../data/opdata4.jld2"), "inputs")
inputs5 = load(joinpath(@__DIR__, "../data/opdata5.jld2"), "inputs")

rewarddist = db.reward
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist))
northstart = HIPPO.ind2pos(mapsize, db.ID2grid[40607])
southstart = HIPPO.ind2pos(mapsize, db.ID2grid[19064])
weststart = HIPPO.ind2pos(mapsize, db.ID2grid[28390])
northeaststart = HIPPO.ind2pos(mapsize, db.ID2grid[45650])

robotinit = northstart
maxbatt = 1000

target = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])
#target = [25,30]
sinit = FullState(robotinit, target, vec(trues(mapsize)), maxbatt) #rand(initialstate(msim))
sinitBasic = BasicState(sinit.robot,sinit.target)


pomdp = FullPOMDP(sinit,
                  size=mapsize,
                  rewarddist=rewarddist,
                  maxbatt=maxbatt)
basic_problem = BasicPOMDP(sinitBasic, size=mapsize)
#mdp_solver = ValueIterationSolver() # creates the solver
#mdp_policy = solve(mdp_solver, UnderlyingMDP(basic_problem))

p = FunctionPolicy(FixedPolicy())
#mdprollout = FORollout(TargetSearchMDPPolicy(mdp_policy))
funcrollout = FORollout(p)

#solver = POMCPSolver(estimate_value = mdprollout, tree_queries=10000, max_time=0.2, c=5) # mdp policy rollout
#solver = POMCPSolver(estimate_value = funcrollout, tree_queries=10000, max_time=0.2, c=5) # up rollout
solver = POMCPSolver(tree_queries=10_000, max_time=0.2, c=5) # random

planner = solve(solver, pomdp)

b0 = initialstate(pomdp)
N = 10000
particle_up = BootstrapFilter(pomdp, N)
particle_b = initialize_belief(particle_up, b0)

hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, sinit=sinit, dt=1/10, max_iter=10, display=false)
r_total, hist = simulateHIPPO(hipposim)

histvec = []
rvec = []
targetfound = Bool[]
newtarget = HIPPO.newtarget(mapsize, db)
for i in 1:2
    hist, r_total = simulateHIPPO(hipposim)
    newtarget = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])
    pomdp = FullPOMDP(sinit,
                  size=mapsize,
                  rewarddist=rewarddist,
                  maxbatt=maxbatt)
    hipposim.sinit = FullState(sinit.robot, newtarget, sinit.visited, sinit.battery)
    hipposim.msim = pomdp
    push!(targetfound, last(hist).s.robot == newtarget)
    push!(histvec, hist)
    push!(rvec, r_total)
end

#hist = simulateBaseline(bsim, sortedpoints)

