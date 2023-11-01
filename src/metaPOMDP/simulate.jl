Base.@kwdef mutable struct HIPPOSimulator 
    msim::TargetSearchPOMDP
    planner::POMCPPlanner
    up::BasicParticleFilter
    b::ParticleCollection
    sinit::TSState
    rewardframes::Frames   = Frames(MIME("image/png"))
    belframes::Frames      = Frames(MIME("image/png"))
    dt::Float64            = 1/10
    max_iter::Int          = 500
    display::Bool          = false
    verbose::Bool          = true
    logging::Bool          = true
    anim::Bool             = false
end

mutable struct PachSimulator 
    msim::RewardPOMDP
    planner::POMCPPlanner
    up::BasicParticleFilter
    b::ParticleCollection
    sinit::RewardState
end

#function HIPPOSimulator(msim::TargetSearchPOMDP, planner::POMCPPlanner, up::BasicParticleFilter, b::ParticleCollection, sinit::TSState; 
#                max_fps=10, gif_fps=10, max_iter=500) 
#    return HIPPOSimulator(msim, planner, up, b, sinit, Frames(MIME("image/png"), fps=gif_fps), Frames(MIME("image/png"), fps=gif_fps), 1/max_fps, max_iter)
#end

function remove_rewards(pomdp, s)
    pomdp.reward[rewardinds(pomdp,s)...] = 0.0
end

function convertinds(m::TargetSearchPOMDP, pos::Vector{Int})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    return inds
end

function generatelocation(m::TargetSearchPOMDP, atraj, robotloc)
    loctraj = Vector{Vector{Int}}(undef, length(atraj))
    currloc = robotloc
    for i in eachindex(loctraj)
        newrobot = bounce(m, currloc, actiondir[atraj[i]])
        loctraj[i] = newrobot
        currloc = newrobot
    end
    loctraj = [convertinds(m, loc) for loc in loctraj]
    return loctraj
end

function loctostr(locvec)
    return [join(locvec[i], ',') for i in eachindex(locvec)]
end



function simulateHIPPO(sim::HIPPOSimulator)
    (;msim,max_iter) = sim 
    r_total = 0.0
    s = sim.sinit
    sp = nothing 
    finalstate = nothing
    o = nothing
    r = 0.0
    iter = 0
    d = 1.0
    b = sim.b 
    bp = sim.b
    a = :nothing 
    info = nothing
    history = NamedTuple[]
    while !isterminal(msim, s) && iter < max_iter
        tm = time()
        #_, info = action_info(sim.planner, sim.b, tree_in_info = true)
        #tree = info[:tree] # maybe set POMCP option tree_in_info = true
        #a_traj = extract_trajectory(root(tree), 5)
        #a = first(a_traj)
        try 
            a, info = action_info(sim.planner, b, tree_in_info = true)
        catch
            #a = :stay
            @warn "POMCP failed to find an action"
            push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
            return history, r_total, iter
        end
        remove_rewards(msim, s.robot) # remove reward at current state
        #display(msim.reward)
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(sim.up, b, a, o)
        (sim.anim || sim.display) && (belframe = render(msim, (sp=sp, bp=b)))
        (sim.anim || sim.display) && (rewardframe = render(msim, (sp=sp, bp=b), true))
        #display(belframe)
        sim.display && display(rewardframe)
        sleep_until(tm += sim.dt)
        iter += 1
        println(iter,"- | s: ", s.robot, " | sp:", sp.robot, " | r:", r, " | o: ", o)
        #println(iter,"- | battery: ", sp.battery, " | dist_to_home: ", dist(sp.robot, msim.robot_init), " | s: ", sp.robot)
        sim.anim && push!(sim.rewardframes, rewardframe)
        sim.anim && push!(sim.belframes, belframe)
        #sim.logging && push!(history, (s=s, a=a, sp=sp, o=o, r=r, bp=b, info=info))
        sim.logging && push!(history, (s=s,a=a))
        finalstate = s
        s = sp
    end
    !sim.logging && push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
    #!sim.logging && push!(history, (a=a,))
    return history, r_total, iter, sim.rewardframes, sim.belframes
end

function predicted_path(sim::PachSimulator)
    (;msim,planner,sinit,b,up) = sim
    s = sinit

    _, info = action_info(planner, b, tree_in_info = true)
    tree = info[:tree] # maybe set POMCP option tree_in_info = true
    a_traj = extract_trajectory(root(tree), 12)
    println(a_traj)
    a = first(a_traj)

    remove_rewards(msim, s.robot) # remove reward at current state
    
    sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
    b = update(up, b, a, o)

    #rewardframe = render(msim, (sp=sp, bp=b), true)
    #display(rewardframe)
    
    s = sp

    return loctostr(generatelocation(msim, a_traj, sinit.robot)), b, s
end

function customsim(msolve::TargetSearchPOMDP, msim::TargetSearchPOMDP, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 10
    dt = 1/max_fps
    d = 1.0
    sim_states = TSState[]
    #frames1 = []
    rewardframes = Frames(MIME("image/png"), fps=10)
    belframes = Frames(MIME("image/png"), fps=10)
    while !isterminal(msim, s) && iter < 500
        tm = time()
        _, info = action_info(planner, b, tree_in_info = true)
        tree = info[:tree] # maybe set POMCP option tree_in_info = true
        a_traj = extract_trajectory(root(tree), 5)
        println(a_traj)
        a = first(a_traj)
        remove_rewards(msim, s) # remove reward at current state
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        belframe = render(msim, (sp=sp, bp=b))
        rewardframe = render(msim, (sp=sp, bp=b), true)
        display(rewardframe)
        sleep_until(tm += dt)
        iter += 1
        #println(iter,"- | s: ", s, " | sp:", sp, " | r:", r, " | o: ", o)
        if iter > 1000
            roi_states = [[1,9],[1,10],[1,8]]
            probs = [0.8,0.8,0.8]
            roi_points = Dict(roi_states .=> probs)
            msolve.rois = roi_points
            planner = solve(solver, msolve)
        end
        push!(sim_states, sp)
        push!(belframes, belframe)
        push!(rewardframes, rewardframe)

        s = sp
        #push!(frames2, render(msim, (sp=s, bp=b), true))
        #if isterminal(msim, s)
        #    break
        #end
    end
    return r_total, sim_states, rewardframes, belframes
end

function beliefsim(msolve::TargetSearchPOMDP, msim::TargetSearchPOMDP, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 10
    dt = 1/max_fps
    d = 1.0
    sim_states = TSState[]
    #frames1 = []
    rewardframes = Frames(MIME("image/png"), fps=10)
    belframes = Frames(MIME("image/png"), fps=10)
    while !isterminal(msim, s) && iter < 500
        tm = time()
        _, info = action_info(planner, b, tree_in_info = true)
        tree = info[:tree] # maybe set POMCP option tree_in_info = true
        a_traj = extract_trajectory(root(tree), 5)
        println(a_traj)
        a = first(a_traj)
        remove_rewards(msim, s) # remove reward at current state
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        b = (1-α) * b + α * msim.reward


        belframe = render(msim, (sp=sp, bp=b))
        rewardframe = render(msim, (sp=sp, bp=b), true)
        display(rewardframe)
        sleep_until(tm += dt)
        iter += 1
        #println(iter,"- | s: ", s, " | sp:", sp, " | r:", r, " | o: ", o)
        if iter > 1000
            roi_states = [[1,9],[1,10],[1,8]]
            probs = [0.8,0.8,0.8]
            roi_points = Dict(roi_states .=> probs)
            msolve.rois = roi_points
            planner = solve(solver, msolve)
        end
        push!(sim_states, sp)
        push!(belframes, belframe)
        push!(rewardframes, rewardframe)

        s = sp
        #push!(frames2, render(msim, (sp=s, bp=b), true))
        #if isterminal(msim, s)
        #    break
        #end
    end
    return r_total, sim_states, rewardframes, belframes
end