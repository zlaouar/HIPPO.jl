using HIPPO
using POMDPs
using POMDPTools
using BasicPOMCP
using DiscreteValueIteration
using ParticleFilters
using JLD2

function results(file)
    data = load(file)
    find_ratio_hippo = sum(data["hippo_targetfound_vec"]) ./ length(data["hippo_targetfound_vec"])
    find_ratio_baseline = sum(data["baseline_targetfound_vec"]) ./ length(data["baseline_targetfound_vec"])
    reward_time_hippo = mean(data["hippo_reward_vec"] ./ data["hippo_time"])
    reward_time_baseline = mean(data["baseline_reward_vec"] ./ data["baseline_time"])
    println("hippo -- rtf: ", reward_time_hippo, " |  find ratio: ", find_ratio_hippo)
    println("baseline -- rtf: ", reward_time_baseline, " |  find ratio: ", find_ratio_baseline)
end

#println("ARGS: ", ARGS)

#mapid = parse(Int, ARGS[1]) 
#startid = parse(Int,ARGS[2])

mapid = 1
startid = 1

filevec = ["../data/opdata1.jld2", 
           "../data/opdata2.jld2",
           "../data/opdata3.jld2",
           "../data/opdata4.jld2",
           "../data/opdata5.jld2"]

#file = filevec[mapid]
file = filevec[1]
inputs = jldopen(joinpath("data", file), "r")["inputs"]
db = jldopen(joinpath("data", file), "r")["db"]


#db = load(joinpath(@__DIR__, "../data/db.jld2"), "db")
#inputs1 = load(joinpath(@__DIR__, "../data/opdata1.jld2"), "inputs")
#inputs2 = load(joinpath(@__DIR__, "../data/opdata2.jld2"), "inputs")
#inputs3 = load(joinpath(@__DIR__, "../data/opdata3.jld2"), "inputs")
#inputs4 = load(joinpath(@__DIR__, "../data/opdata4.jld2"), "inputs")
#inputs5 = load(joinpath(@__DIR__, "../data/opdata5.jld2"), "inputs")

rewarddist = db.reward
rewarddist = abs.(rewarddist)
mapsize = reverse(size(rewarddist))
northstart = HIPPO.ind2pos(mapsize, db.ID2grid[40607])
southstart = HIPPO.ind2pos(mapsize, db.ID2grid[19064])
weststart = HIPPO.ind2pos(mapsize, db.ID2grid[28390])
northeaststart = HIPPO.ind2pos(mapsize, db.ID2grid[45650])

startvec = [northstart, southstart, weststart, northeaststart]

robotinit = startvec[startid]
maxbatt = 1000

target = HIPPO.newtarget(mapsize, db)
#target = [25,30]
sinit = UnifiedState(robotinit, HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()]), vec(trues(mapsize)), maxbatt, false, :up)#rand(initialstate(msim))


cam_info = HIPPO.CameraInfo(
    deg2rad(71.5), # horizontal fov
    deg2rad(56.8), # vertical fov
)

pomdp = UnifiedPOMDP(sinit, 
                    size=mapsize, 
                    rewarddist=rewarddist, 
                    maxbatt=maxbatt,
                    camera_info=cam_info)

greedyrollout = FORollout(GreedyPolicy(pomdp))

#solver = POMCPSolver(estimate_value = mdprollout, tree_queries=10000, max_time=0.2, c=5) # mdp policy rollout
#solver = POMCPSolver(estimate_value = funcrollout, tree_queries=10000, max_time=0.2, c=5) # up rollout
solver = POMCPSolver(tree_queries=10_000, max_time=0.2, c=100) # random

planner = solve(solver, pomdp)

b0 = initialstate(pomdp)
N = 20000
particle_up = BootstrapFilter(pomdp, N)
particle_b = initialize_belief(particle_up, b0)

bsim = MapBaselineSimulator(msim=pomdp, sinit=sinit, up=particle_up, b=particle_b, 
                            dt=1/4, max_iter=maxbatt, display=true, verbose=true)
baseline_hist, baseline_rtot = simulateBaseline(bsim)
