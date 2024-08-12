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

function results_avg(files)
    datavec = []
    for file in files
        data = load(file)
        push!(datavec, data)
    end
    hippo_reward_vec = vcat(get.(datavec, "hippo_reward_vec", 1)...)
    hippo_time_vec = vcat(get.(datavec, "hippo_time", 1)...)
    hippo_targetfound_vec = vcat(get.(datavec, "hippo_targetfound_vec", 1)...)

    baseline_reward_vec = vcat(get.(datavec, "baseline_reward_vec", 1)...)
    baseline_time_vec = vcat(get.(datavec, "baseline_time", 1)...)
    baseline_targetfound_vec = vcat(get.(datavec, "baseline_targetfound_vec", 1)...)

    sims = length(hippo_reward_vec)
    println("sims per file: ", sims)
    hippo_rtf = hippo_reward_vec ./ hippo_time_vec
    hippo_rtf_mean = mean(hippo_rtf)
    hippo_rtf_std = std(hippo_rtf) / sqrt(sims)
    hippo_targetfound_ratio = sum(hippo_targetfound_vec) / length(hippo_targetfound_vec)

    baseline_rtf = baseline_reward_vec ./ baseline_time_vec
    baseline_rtf = baseline_rtf[broadcast(!, isnan.(baseline_rtf))]
    baseline_rtf_mean = mean(baseline_rtf)
    baseline_rtf_std = std(baseline_rtf) / sqrt(sims)
    baseline_targetfound_ratio = sum(baseline_targetfound_vec) / length(baseline_targetfound_vec)

    println("hippo -- rtf: ", hippo_rtf_mean, " ± ", hippo_rtf_std, " |  find ratio: ", hippo_targetfound_ratio)
    println("baseline -- rtf: ", baseline_rtf_mean, " ± ", baseline_rtf_std, " |  find ratio: ", baseline_targetfound_ratio)
end

println("ARGS: ", ARGS)

mapid = parse(Int, ARGS[1]) 
startid = parse(Int,ARGS[2])
num_sims = parse(Int,ARGS[3])

filevec = ["../data/opdata1.jld2", 
           "../data/opdata2.jld2",
           "../data/opdata3.jld2",
           "../data/opdata4.jld2",
           "../data/opdata5.jld2"]

file = filevec[mapid]

function runsims(file, startid, num_sims)
    inputs = load(joinpath(@__DIR__, file), "inputs")
    db = load(joinpath(@__DIR__, file), "db")


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

    opdata = inputs
    pospoints = HIPPO.getdata(opdata, db, mapsize)#[end-10:end]
    polypoints = HIPPO.polypoints(opdata, db, mapsize)#[1:2]
    sortedpoints = HIPPO.gendists(pospoints, robotinit)

    target = HIPPO.newtarget(mapsize, db)
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

    hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, 
                    sinit=sinit, dt=1/10, max_iter=maxbatt, display=false, logging=false)
    bsim = BaselineSimulator(msim=pomdp, sinit=sinit, dt=1/4, max_iter=maxbatt, 
                    display=false, verbose=false, logging=false)

    hippo_hist_vec = []
    hippo_reward_vec = []
    hippo_targetfound_vec = Bool[]
    hippo_time_vec = []
    baseline_hist_vec = []
    baseline_reward_vec = []
    baseline_targetfound_vec = Bool[]
    baseline_time_vec = []
    #newtarget = HIPPO.newtarget(mapsize, db)
    newtarget = sinit.target
    #num_sims = 10
    for i in 1:num_sims
        println("HIPPO sim: ", i, " of ", num_sims)
        hippo_hist, hippo_rtot, hippo_time = simulateHIPPO(hipposim)
        push!(hippo_time_vec, hippo_time)
        push!(hippo_targetfound_vec, last(hippo_hist).s.robot == newtarget)
        println("HIPPO target and last state: ", newtarget, " ", last(hippo_hist).s.robot, " | iterations: ", hippo_time)
        push!(hippo_hist_vec, hippo_hist)
        push!(hippo_reward_vec, hippo_rtot)

        pomdp = FullPOMDP(sinit,
            size=mapsize,
            rewarddist=rewarddist,
            maxbatt=maxbatt)
        hipposim.msim = pomdp
        bsim.msim = pomdp

        println("Baseline sim: ", i, " of ", num_sims)
        baseline_hist, baseline_rtot, baseline_time = simulateBaseline(bsim, sortedpoints, polypoints)
        push!(baseline_targetfound_vec, last(baseline_hist).s.robot == newtarget)
        println("Baseline target and last state: ", newtarget, " ", last(baseline_hist).s.robot, " | iterations: ", baseline_time)
        push!(baseline_hist_vec, baseline_hist)
        push!(baseline_reward_vec, baseline_rtot)
        push!(baseline_time_vec, baseline_time)

        newtarget = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])
        newstate = FullState(sinit.robot, newtarget, sinit.visited, sinit.battery)
        pomdp = FullPOMDP(newstate,
                    size=mapsize,
                    rewarddist=rewarddist,
                    maxbatt=maxbatt)
        hipposim.msim = pomdp
        bsim.msim = pomdp

        hipposim.sinit = newstate
        bsim.sinit = newstate
 
    end
    return hippo_hist_vec, hippo_reward_vec, hippo_targetfound_vec, hippo_time_vec,
            baseline_hist_vec, baseline_reward_vec, baseline_targetfound_vec, baseline_time_vec
end



#= jldsave(joinpath(@__DIR__, "../results/results.jld2"), 
            hippo_hist_vec=hippo_hist_vec,
            hippo_reward_vec=hippo_reward_vec,
            hippo_targetfound_vec=hippo_targetfound_vec,
            baseline_hist_vec=baseline_hist_vec,
            baseline_reward_vec=baseline_reward_vec,
            baseline_targetfound_vec=baseline_targetfound_vec)
 =#

hippo_hist_vec, hippo_reward_vec, hippo_targetfound_vec, hippo_time,
            baseline_hist_vec, baseline_reward_vec, baseline_targetfound_vec, baseline_time = runsims(file, startid, num_sims)

jldsave(joinpath(@__DIR__, "../results/", ARGS[4]), 
            hippo_reward_vec=hippo_reward_vec,
            hippo_targetfound_vec=hippo_targetfound_vec,
            baseline_reward_vec=baseline_reward_vec,
            baseline_targetfound_vec=baseline_targetfound_vec,
            hippo_time=hippo_time,
            baseline_time=baseline_time)

