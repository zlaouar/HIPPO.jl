Base.@kwdef mutable struct TargetSearchSim
    num_sims::Int
    simulator::AbstractSimulator
    sinit::TSState
    reward_vec::Vector{Float64} = Vector{Float64}(undef, num_sims)
    targetfound_vec::BitVector = BitVector(undef, num_sims)
    time_vec::Vector{Int} = Vector{Int}(undef, num_sims)
end

# function TargetSearchSim(num_sims, simulator, sinit, reward_vec=Vector{Float64}(undef, num_sims), 
#                             targetfound_vec=BitVector(undef, num_sims), time_vec=Vector{Int}(undef, num_sims))
#     TargetSearchSim(num_sims, simulator, sinit, reward_vec, targetfound_vec, time_vec)
# end
sim_reward(m::TargetSearchPOMDP, s::TSState, a, sp::TSState) = sim_reward(m, sp)

function sim_reward(m::TargetSearchPOMDP, sp::TSState)
    reward_running = -1.0
    reward_target = 0.0

    # if isequal(sp.robot, sp.target)# if target is found
    #     reward_running = 0.0
    #     reward_target = 1000.0 
    #     return reward_running + reward_target
    # end

    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end

    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        #return reward_running + reward_target + m.reward[inds...]*sp.visited[spind] + reward_nogo
        return m.reward[inds...]*sp.visited[spind]
    end
end

function save_data(rtot, hist, targetfound, fname, gname)
    jldopen(fname, "a") do file
        group = JLD2.Group(file, "iter" * string(gname))
        group["hippo_reward_vec"] = rtot
        group["hippo_time"] = length(hist)
        group["hippo_targetfound_vec"] = targetfound
    end
end

function benchmark_planner(sim::TargetSearchSim, rewarddist, newtarget_func, fname; args=(), verbose=false, save=true)
    rm(fname, force=true)
    for i ∈ 1:sim.num_sims
        #verbose && @info "sim: ", i
        hist, rtot = simulate(sim.simulator, custom_reward=true)
        save && save_data(rtot, hist, last(hist).sp.robot == sim.simulator.msim.targetloc, fname, i)        
        verbose && @info "sim: ", i, " | reward: ", rtot, " | time: ", length(hist), "target: ", 
                sim.simulator.msim.targetloc, " | targetfound: ", last(hist).sp.robot == sim.simulator.msim.targetloc
        #sim.reward_vec[i] = rtot
        #sim.targetfound_vec[i] = last(hist).sp.robot == sim.simulator.msim.targetloc
        #sim.targetfound_vec[i] ? println("Target found") : println("Target not found")
        #sim.time_vec[i] = length(hist)
        newtarget = newtarget_func(args...)
        #newtarget = HIPPO.ind2pos(sim.simulator.msim.size, [1, 1])
        reset_pomdp!(sim.simulator, rewarddist, newtarget)
        GC.gc()
    end
    return sim
end

function benchmark_planner(sim::TargetSearchSim, rewarddist, ID2grid, fname, histvec::Vector, verbose=false)
    for i ∈ 1:sim.num_sims
        hist, rtot = simulate(sim.simulator)
        sim.reward_vec[i] = rtot
        sim.targetfound_vec[i] = last(hist).sp.robot == sim.simulator.msim.targetloc
        #sim.targetfound_vec[i] ? println("Target found") : println("Target not found")
        sim.time_vec[i] = length(hist)
        newtarget = HIPPO.newtarget(sim.simulator.msim.size, ID2grid)
        reset_pomdp!(sim, rewarddist, newtarget)
        push!(histvec, hist)
    end
    return sim, histvec
end

function reset_pomdp!(simulator::HIPPOSimulator{<:PachPOMDP}, rewarddist, target)
    pomdp = simulator.msim
    if hasproperty(pomdp, :currentbatt)
        pomdp.currentbatt = copy(simulator.msim.maxbatt)
        simulator.sinit = UnifiedState(pomdp.robot_init, target, vec(trues(pomdp.size...)), 
                                        pomdp.maxbatt, false, pomdp.initial_orientation)
    end
    simulator.sinit = FullState(pomdp.robot_init, target, vec(trues(pomdp.size...)), 
                                    pomdp.maxbatt, pomdp.initial_orientation)
    pomdp.reward = copy(rewarddist)
    pomdp.targetloc = copy(target)
    
    simulator.b = initialize_belief(simulator.up, initialstate(pomdp))
end

function reset_pomdp!(simulator::HIPPOSimulator{<:UnifiedPOMDP}, rewarddist, target)
    pomdp = simulator.msim
    if hasproperty(pomdp, :currentbatt)
        pomdp.currentbatt = copy(simulator.msim.maxbatt)
        simulator.sinit = UnifiedState(pomdp.robot_init, target, vec(trues(pomdp.size...)), 
                                        pomdp.maxbatt, false, pomdp.initial_orientation)
    end
    simulator.sinit = UnifiedState(pomdp.robot_init, target, vec(trues(pomdp.size...)), 
                                    pomdp.maxbatt, false, pomdp.initial_orientation)
    pomdp.reward = copy(rewarddist)
    pomdp.targetloc = copy(target)
    
    simulator.b = initialize_belief(simulator.up, initialstate(pomdp))
end

function reset_pomdp!(simulator::MapBaselineSimulator, rewarddist, target)
    pomdp = simulator.msim
    pomdp.currentbatt = copy(simulator.msim.maxbatt)
    pomdp.reward = copy(rewarddist)
    pomdp.targetloc = copy(target)
    simulator.sinit = UnifiedState(pomdp.robot_init, target, vec(trues(pomdp.size...)), 
                                        pomdp.maxbatt, false, pomdp.initial_orientation)
    simulator.b = initialize_belief(simulator.up, initialstate(pomdp))
end

function reset_pomdp!(simulator::UnifiedBaselineSimulator, rewarddist, target)
    pomdp = simulator.msim
    pomdp.currentbatt = copy(simulator.msim.maxbatt)
    pomdp.reward = copy(rewarddist)
    pomdp.targetloc = copy(target)
    simulator.sinit = UnifiedState(pomdp.robot_init, copy(target), vec(trues(pomdp.size...)), 
                                        pomdp.maxbatt, false, pomdp.initial_orientation)
end

function show_benchmark_results(file::String)
    data = load(file; nested=true)
    data = [collect(values(data[i])) for i ∈ keys(data)]
    targetfound_vec = [sim[1] for sim ∈ data]
    reward_vec = [sim[3] for sim ∈ data]
    time_vec = [sim[2] for sim ∈ data]

    find_ratio = sum(targetfound_vec) / length(targetfound_vec)
    reward_time = mean(reward_vec ./ time_vec)
    time = mean(time_vec)
    println("average reward: ", reward_time, " |  find ratio: ", find_ratio, " |  time: ", time)
end

function show_benchmark_results(files::Vector{String})
    for file ∈ files
        show_benchmark_results(file)
    end
end

# function show_benchmark_results(file)
#     data = load(file)["benchmarks"]

#     for sim ∈ data
#         find_ratio = sum(sim.targetfound_vec) / length(sim.targetfound_vec)
#         reward_time = mean(sim.reward_vec ./ sim.time_vec)
#         time = mean(sim.time_vec)
#         println("rtf: ", reward_time, " |  find ratio: ", find_ratio, " |  time: ", time)
#     end
# end

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