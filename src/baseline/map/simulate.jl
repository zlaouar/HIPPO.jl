Base.@kwdef mutable struct MapBaselineSimulator <: AbstractSimulator
    msim::TargetSearchPOMDP 
    sinit::TSState
    up::BasicParticleFilter
    b::ParticleCollection         
    rewardframes::Frames    = Frames(MIME("image/png"))
    belframes::Frames      = Frames(MIME("image/png"))
    dt::Float64             = 1/10
    max_iter::Int           = 500
    display::Bool           = false
    verbose::Bool           = true
    logging::Bool           = true
end

function simulate(sim::MapBaselineSimulator)
    (;msim,max_iter) = sim
    r_total = 0.0
    d = 1.0
    s = sim.sinit
    b = sim.b
    a = :nothing
    iter = 0
    history = NamedTuple[]
    sim.logging && push!(history, (s=s, a=a, sp=s, bp=b))
    while !isterminal(msim, s, s.target) && iter < max_iter 
        tm = time()
        a = mapaction(msim, b)
        sim.verbose && println(iter,"- | s: ", s.robot, " | a: ", a, " | mode: ", mode(b).target)
        #remove_rewards(msim, s.robot) # remove reward at current state
        sp, o, r = @gen(:sp, :o, :r)(msim, s, a)
        isterminal(msim, sp) && break
        b = update(sim.up, b, a, o)
        # try
        #     b = update(sim.up, b, a, o)
        # catch e
        #     @error "Error updating belief: $e"
        #     @info a, o 
        #     return history, r_total 
        # end

        #sim.verbose && println(iter,"- | s: ", s.robot, " | human: ", s.human_in_fov, " | orient: ", s.orientation, " | sbatt: ", s.battery, " | a: ", a, 
        #" | sp_robot:", sp.robot, " | sp_target:", sp.target, " | spbatt: ", sp.battery, " | r:", r, " | o: ", o)
        #newrobot = bounce(msim, s.robot, actiondir[a])
        #s = UnifiedState(newrobot, s.target, vec(trues(size(msim.reward))), s.battery-1, s.human_in_fov, s.orientation)
        r_total += d*r
        #remove_rewards(msim, s.robot) # remove reward at current state
        d *= discount(msim)
        sim.display && (belframe = render(msim, (sp=sp, bp=b)))
        sim.display && (rewardframe = render(msim, (sp=sp, bp=b), true))
        #sim.anim && push!(sim.rewardframes, rewardframe)
        #sim.anim && push!(sim.belframes, belframe)
        sim.display && display(rewardframe) 
        sim.display && sleep_until(tm += sim.dt)
        sim.logging && push!(history, (s=s, sp=sp, a=a, bp=b))
        s = sp
        iter += 1
    end
    #!sim.logging && push!(history, (s=s, a=a, sp=sp, bp=b))
    return history, r_total, iter, sim.rewardframes
end