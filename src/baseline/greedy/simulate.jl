Base.@kwdef mutable struct UnifiedBaselineSimulator 
    msim::TargetSearchPOMDP 
    sinit::TSState         
    rewardframes::Frames    = Frames(MIME("image/png"))
    dt::Float64             = 1/10
    max_iter::Int           = 500
    display::Bool           = false
    verbose::Bool           = true
    logging::Bool           = true
end

function simulateBaseline(sim::UnifiedBaselineSimulator)
    (;msim,max_iter) = sim
    r_total = 0.0
    d = 1.0
    s = sim.sinit
    a = :nothing
    iter = 0
    history = NamedTuple[]
    while !isterminal(msim, s, s.target) && iter < max_iter 
        tm = time()
        a = action(msim, s)
        #remove_rewards(msim, s.robot) # remove reward at current state
        newrobot = bounce(msim, s.robot, actiondir[a])
        s = UnifiedState(newrobot, s.target, vec(trues(size(msim.reward))), s.battery-1, s.human_in_fov, s.orientation)
        r_total += d*msim.reward[rewardinds(msim, s.robot)...]
        remove_rewards(msim, s.robot) # remove reward at current state
        d *= discount(msim)
        rewardframe = render(msim, (sp=s,), true)
        sim.display && display(rewardframe) 
        sim.display && sleep_until(tm += sim.dt)
        iter += 1
        push!(sim.rewardframes, rewardframe)
        sim.logging && push!(history, (s=s, a=a))
    end
    !sim.logging && push!(history, (s=s, a=a))
    return history, r_total, iter, sim.rewardframes
end