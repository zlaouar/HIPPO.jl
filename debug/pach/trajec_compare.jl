using HIPPO
using POMDPs
using POMDPTools
using BasicPOMCP
using DiscreteValueIteration
using ParticleFilters
using JLD2
using Reel

function rendertrajec(m, hippo_hist, baseline_hist)
    hippovec = [state.s.robot for state ∈ hippo_hist]
    baselinevec = [state.s.robot for state ∈ baseline_hist]
    display(render(m, hippo_hist[end].s.target, hippovec, baselinevec))
end

# mapid = parse(Int, ARGS[1])
mapid = 1
startid = 1
# startid = parse(Int,ARGS[2])

filevec = ["../data/opdata1.jld2", 
           "../data/opdata2.jld2",
           "../data/opdata3.jld2",
           "../data/opdata4.jld2",
           "../data/opdata5.jld2"]

file = filevec[mapid]


inputs = load(joinpath(@__DIR__, file), "inputs")
db = load(joinpath(@__DIR__, file), "db")

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

opdata = inputs
pospoints = HIPPO.getdata(opdata, db, mapsize)#[end-10:end]
polypoints = HIPPO.polypoints(opdata, db, mapsize)#[1:2]
sortedpoints = HIPPO.gendists(pospoints, robotinit)

target = HIPPO.newtarget(mapsize, db)
target = [49,28]
#arget = [25,30]
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
N = 25000
particle_up = BootstrapFilter(pomdp, N)
particle_b = initialize_belief(particle_up, b0)

hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, 
                sinit=sinit, dt=1/10, max_iter=maxbatt, display=false, logging=false,
                rewardframes=Frames(MIME("image/png"), fps=15), belframes=Frames(MIME("image/png"), fps=15))
bsim = BaselineSimulator(msim=pomdp, sinit=sinit, dt=1/4, max_iter=maxbatt, 
                display=false, verbose=false, logging=false,
                rewardframes=Frames(MIME("image/png"), fps=15))
#newtarget = HIPPO.newtarget(mapsize, db)
newtarget = sinit.target
#num_sims = 10




hippo_hist, hippo_rtot, hippo_time, hippoframes, belframes = simulateHIPPO(hipposim)
println("HIPPO target and last state: ", newtarget, " ", last(hippo_hist).s.robot, " | iterations: ", hippo_time)

begin
    pomdp = FullPOMDP(sinit,
        size=mapsize,
        rewarddist=rewarddist,
        maxbatt=maxbatt)
    hipposim.msim = pomdp
    bsim.msim = pomdp

    baseline_hist, baseline_rtot, baseline_time, baseframes = simulateBaseline(bsim, sortedpoints, polypoints)
    println("Baseline target and last state: ", newtarget, " ", last(baseline_hist).s.robot, " | iterations: ", baseline_time)
end
# write("hippo.gif", hippoframes)
# write("baseline.gif", baseframes)
# write("hippobelief.gif", belframes)



#= jldsave(joinpath(@__DIR__, "../results/results.jld2"), 
            hippo_hist_vec=hippo_hist_vec,
            hippo_reward_vec=hippo_reward_vec,
            hippo_targetfound_vec=hippo_targetfound_vec,
            baseline_hist_vec=baseline_hist_vec,
            baseline_reward_vec=baseline_reward_vec,
            baseline_targetfound_vec=baseline_targetfound_vec)
 =#



# jldsave(joinpath(@__DIR__, "../results/", ARGS[4]), 
#             hippo_reward_vec=hippo_reward_vec,
#             hippo_targetfound_vec=hippo_targetfound_vec,
#             baseline_reward_vec=baseline_reward_vec,
#             baseline_targetfound_vec=baseline_targetfound_vec,
#             hippo_time=hippo_time,
#             baseline_time=baseline_time)

