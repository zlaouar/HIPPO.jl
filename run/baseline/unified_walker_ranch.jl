using Pkg

Pkg.activate(".")

using HIPPO
using POMDPs
using POMDPTools
using ParticleFilters
using StaticArrays
using BasicPOMCP
using JLD2

db = load("data/db.jld2", "db")

rewarddist = db.reward
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist))
northstart = HIPPO.ind2pos(mapsize, db.ID2grid[40607])
southstart = HIPPO.ind2pos(mapsize, db.ID2grid[19064])
weststart = HIPPO.ind2pos(mapsize, db.ID2grid[28390])
northeaststart = HIPPO.ind2pos(mapsize, db.ID2grid[45650])

robotinit = northstart
maxbatt = 1000

sinit = UnifiedState(robotinit, HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()]), vec(trues(mapsize)), maxbatt, false, :up)#rand(initialstate(msim))

cam_info = HIPPO.CameraInfo(
    deg2rad(71.5), # horizontal fov
    deg2rad(56.8), # vertical fov
)

pomdp = UnifiedPOMDP(sinit, 
                    size=mapsize, 
                    rewarddist=rewarddist, 
                    maxbatt=maxbatt,
                    options=Dict(:observation_model=>:falco),
                    rollout_depth=maxbatt,
                    camera_info=cam_info)

p = FunctionPolicy(FixedPolicy())
greedyrollout = FORollout(GreedyPolicy(pomdp))
funcrollout = FORollout(p)
#mdprollout = FORollout(mdp_policy) # change MDP reward mat to pompdp reward mat
#solver = POMCPSolver(estimate_value = mdprollout, tree_queries=10000, max_time=0.2, c=5) # mdp policy rollout
#solver = POMCPSolver(estimate_value = funcrollout, tree_queries=10000, max_time=0.2, c=5) # up rollout
#solver = POMCPSolver(tree_queries=10_000, max_time=0.2, c=100) # random
solver = POMCPSolver(estimate_value=greedyrollout,tree_queries=10_000, max_time=0.2, c=100) # random


planner = solve(solver,pomdp)


b0 = initialstate(pomdp)
N = 10000
particle_up = BootstrapFilter(pomdp, N)
particle_b = initialize_belief(particle_up, b0)

#a, info = action_info(planner, Deterministic(FullState([13,4],mapsize,vec(trues(mapsize)), maxbatt)), tree_in_info=true)
#inchrome(D3Tree(info[:tree], init_expand=3))

hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, sinit=sinit, dt=1/10, max_iter=maxbatt, display=true)
#hist, r_total = simulateHIPPO(hipposim)

#renderVIPolicy(mdp_policy, basic_pomdp, sinitBasic) # render MDP policy


#println("Total Reward: ", r_total)                   

bsim = UnifiedBaselineSimulator(msim=pomdp, sinit=sinit, dt=1/10, max_iter=maxbatt, display=true)
hist, rtot = simulateBaseline(bsim)
# histvec = []
# rvec = []
# targetfound = Bool[]
# newtarget = HIPPO.newtarget(mapsize, db)
# for i in 1:2
#     hist, rtot = simulateBaseline(bsim, sortedpoints, polypoints)
#     newtarget = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])
#     pomdp = FullPOMDP(sinit,
#                   size=mapsize,
#                   rewarddist=rewarddist,
#                   maxbatt=maxbatt)
#     bsim.msim = pomdp
#     bsim.sinit = FullState(sinit.robot, newtarget, sinit.visited, sinit.battery)
#     push!(targetfound, last(hist).s.robot == newtarget)
#     push!(histvec, hist)
#     push!(rvec, rtot)
# end

#hist = simulateBaseline(bsim, sortedpoints)

