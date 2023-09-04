using HIPPO
using POMDPs
using POMDPTools
using BasicPOMCP
using DiscreteValueIteration
using ParticleFilters
using JLD2

function run_test()
    db = load(joinpath(@__DIR__, "../data/db.jld2"), "db")
    inputs1 = load(joinpath(@__DIR__, "../data/opdata1.jld2"), "inputs")
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

    robotinit = northstart
    maxbatt = 1000

    opdata = inputs1
    pospoints = HIPPO.getdata(opdata, db, mapsize)#[end-10:end]
    polypoints = HIPPO.polypoints(opdata, db, mapsize)#[1:2]
    sortedpoints = HIPPO.gendists(pospoints, robotinit)

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

    hipposim = HIPPOSimulator(msim=pomdp, planner=planner, up=particle_up, b=particle_b, sinit=sinit, dt=1/10, max_iter=maxbatt, display=false)
    bsim = BaselineSimulator(msim=pomdp, sinit=sinit, dt=1/4, max_iter=maxbatt, display=false, verbose=false)

    hippo_hist_vec = []
    hippo_reward_vec = []
    hippo_targetfound_vec = Bool[]
    baseline_hist_vec = []
    baseline_reward_vec = []
    baseline_targetfound_vec = Bool[]
    newtarget = HIPPO.newtarget(mapsize, db)
    num_sims = 10
    for i in 1:num_sims
        hippo_hist, hippo_rtot = simulateHIPPO(hipposim)

        pomdp = FullPOMDP(sinit,
            size=mapsize,
            rewarddist=rewarddist,
            maxbatt=maxbatt)
        hipposim.msim = pomdp
        bsim.msim = pomdp

        baseline_hist, baseline_rtot = simulateBaseline(bsim, sortedpoints, polypoints)
        
        push!(hippo_targetfound_vec, last(hippo_hist).s.robot == newtarget)
        push!(baseline_targetfound_vec, last(baseline_hist).s.robot == newtarget)

        push!(hippo_hist_vec, hippo_hist)
        push!(hippo_reward_vec, hippo_rtot)

        push!(baseline_hist_vec, baseline_hist)
        push!(baseline_reward_vec, baseline_rtot)

        pomdp = FullPOMDP(sinit,
                    size=mapsize,
                    rewarddist=rewarddist,
                    maxbatt=maxbatt)
        hipposim.msim = pomdp
        bsim.msim = pomdp

        newstate = FullState(sinit.robot, newtarget, sinit.visited, sinit.battery)
        hipposim.sinit = newstate
        bsim.sinit = newstate



        newtarget = HIPPO.ind2pos(mapsize, db.ID2grid[HIPPO.chooseTrueCell()])

    end
end

hippo_hist_vec, hippo_reward_vec, hippo_targetfound_vec, 
baseline_hist_vec, baseline_reward_vec, baseline_targetfound_vec = run_test()

jldsave(joinpath(@__DIR__, "../results/results.jld2"), 
            hippo_hist_vec=hippo_hist_vec,
            hippo_reward_vec=hippo_reward_vec,
            hippo_targetfound_vec=hippo_targetfound_vec,
            baseline_hist_vec=baseline_hist_vec,
            baseline_reward_vec=baseline_reward_vec,
            baseline_targetfound_vec=baseline_targetfound_vec)