Base.@kwdef mutable struct UnifiedBaselineSimulator <: AbstractSimulator
    msim::TargetSearchPOMDP 
    sinit::TSState         
    rewardframes::Frames    = Frames(MIME("image/png"))
    dt::Float64             = 1/10
    max_iter::Int           = 500
    display::Bool           = false
    verbose::Bool           = true
    logging::Bool           = true
end

function simulate(sim::UnifiedBaselineSimulator; custom_reward=false)
    (;msim,max_iter) = sim
    r_total = 0.0
    d = 1.0
    s = sim.sinit
    sp = s
    a = :nothing
    iter = 0
    history = NamedTuple[]
    sim.logging && push!(history, (s=s, a=a, sp=sp))
    while !isterminal(msim, s) && iter < max_iter 
        tm = time()
        a = action(msim, s)
        #remove_rewards(msim, s.robot) # remove reward at current state
        newrobot = bounce(msim, s.robot, actiondir[a])
        sp = UnifiedState(newrobot, s.target, vec(trues(size(msim.reward))), s.battery-1, s.human_in_fov, s.orientation)
        isterminal(msim, sp) && break

        if custom_reward
            r = sim_reward(msim, s, a, sp)
        else
            r = msim.reward[rewardinds(msim, sp.robot)...]
        end
        
        sim.verbose && println(iter,"- | s: ", s.robot, " | target: ", s.target, " | a: ", a, " | r: ", r)
        r_total += d*r
        remove_rewards(msim, sp.robot) # remove reward at current state
        d *= discount(msim)

        if sim.display
            rewardframe = render(msim, (sp=sp,), true)
            display(rewardframe) 
            sleep_until(tm += sim.dt)
            push!(sim.rewardframes, rewardframe)
        end

        sim.logging && push!(history, (s=s, a=a, sp=sp))
        s = sp
        iter += 1
    end
    #!sim.logging && push!(history, (s=s, a=a, sp=sp))
    return history, r_total
end