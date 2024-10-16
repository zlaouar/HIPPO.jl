Base.@kwdef mutable struct HIPPOSimulator{POMDP_TYPE} <: AbstractSimulator
    msim::POMDP_TYPE
    planner::POMCPPlanner
    up::BasicParticleFilter
    b::ParticleCollection
    sinit::TSState
    dt::Float64            = 1/10
    rewardframes::Frames   = Frames(MIME("image/png"), fps=1/dt)
    belframes::Frames      = Frames(MIME("image/png"), fps=1/dt)
    max_iter::Int          = 500
    display::Bool          = false
    verbose::Bool          = true
    logging::Bool          = true
    anim::Bool             = false
end

mutable struct FlightParams
    flight_mode::String
    desired_agl_alt::Float64
    max_speed::Float64
    home_location::Vector{Float64}
    target_bias::Float64
end
mutable struct PachSimulator 
    msim::TargetSearchPOMDP
    planner::POMCPPlanner
    up::BasicParticleFilter
    b::ParticleCollection
    sinit::TSState
    location_dict::Dict{String, Vector{Float64}}
    previous_action::Symbol
    previous_state::TSState
    waypointID::Int
    flight_params::FlightParams
    latest_obs::Symbol
    replan_flag::Bool
end

#function HIPPOSimulator(msim::TargetSearchPOMDP, planner::POMCPPlanner, up::BasicParticleFilter, b::ParticleCollection, sinit::TSState; 
#                max_fps=10, gif_fps=10, max_iter=500) 
#    return HIPPOSimulator(msim, planner, up, b, sinit, Frames(MIME("image/png"), fps=gif_fps), Frames(MIME("image/png"), fps=gif_fps), 1/max_fps, max_iter)
#end

function remove_rewards(pomdp, s)
    pomdp.reward[rewardinds(pomdp,s)...] = 0.0
end

function convertinds(m::TargetSearchPOMDP, pos::Union{Vector{Int},SVector{2,Int}})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    return inds
end

function inertial_to_mat_inds(size::Union{Vector{Int},Tuple}, pos::Union{Vector{Int},Tuple})
    correct_ind = reverse(pos)
    xind = size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    return inds
end

function mat_to_inertial_inds(size::Union{SVector{2, Int64},Vector{Int},Tuple}, 
                              pos::Union{SVector{2, Int64},Vector{Int},Tuple})
    size = reverse(size)
    yind = size[1]+1 - pos[1]
    inds = [pos[2], yind]
    return inds
end

"""
    Given a reference lat/lon point, this function will find the closest relevant point within the provided database
"""
function find_closest_grid_point(location_dict, point)
    coord_set = collect(values(location_dict))
    grid_set = collect(keys(location_dict))
    coord_set = [loc[1:2] for loc ∈ coord_set]

    if point[1] < 0.0
        point_coord = [point[2], point[1]]
    else
        point_coord = [point[1], point[2]]
    end

    coord_set = [loc - point_coord for loc ∈ coord_set]
    tot_dist = [loc[1]^2 + loc[2]^2 for loc ∈ coord_set]

    _, i = findmin(tot_dist)
    closest_point = grid_set[i]

    closest_point = split(closest_point, ',')
    closest_point = parse.(Int, closest_point)
    
    return closest_point
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

function simulate(sim::HIPPOSimulator{<:PachPOMDP}; custom_reward=false)
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
    a = Union{Symbol, Vector{Int64}}[:nothing]
    info = nothing
    history = NamedTuple[]
    (sim.anim || sim.display) && (belframe = render(msim, (sp=s, bp=bp)))
    sim.display && display(belframe)
    sim.logging && push!(history, (s=s, a=a, sp=s, bp=bp, info=info))
    while !isterminal(msim, s) && iter < max_iter
        tm = time()
        #_, info = action_info(sim.planner, sim.b, tree_in_info = true)
        #tree = info[:tree] # maybe set POMCP option tree_in_info = true
        #a_traj = extract_trajectory(root(tree), 5)
        #a = first(a_traj)
        #try 
        a, _ = action_info(sim.planner, b)

        
        #catch e
            #a = :stay
        #    @warn "POMCP failed to find an action: ", e
        #    push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
        #    return history, r_total, iter
        #end
        remove_rewards(msim, s.robot) # remove reward at current state
        #display(msim.reward)
        if custom_reward
            sp, o = @gen(:sp, :o)(msim, s, a)
            r = sim_reward(msim, s, a, sp)
        else
            sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        end

        isterminal(msim, sp) && break
        
        r_total += d*r
        d *= discount_factor(msim)
        bp = update(sim.up, b, a, o)
        (sim.anim || sim.display) && (belframe = render(msim, (sp=sp, bp=bp)))
        (sim.anim || sim.display) && (rewardframe = render(msim, (sp=sp, bp=bp), true))
        #display(belframe)
        sim.display && display(belframe)
        sim.verbose && println(iter,"- | s: ", s.robot, " | orient: ", s.orientation, " | sbatt: ", s.battery, " | a: ", a, 
        " | sp_robot:", sp.robot, " | sp_target:", sp.target, " | spbatt: ", sp.battery, " | r:", r, " | o: ", o)
        #println(iter,"- | battery: ", sp.battery, " | dist_to_home: ", dist(sp.robot, msim.robot_init), " | s: ", sp.robot)
        sim.anim && push!(sim.rewardframes, rewardframe)
        sim.anim && push!(sim.belframes, belframe)
        sim.logging && push!(history, (s=s, a=a, sp=sp, o=o, r=r, bp=bp, info=info))
        #sim.logging && push!(history, (s=s,a=a))
        finalstate = s
        s = sp
        b = bp
        iter += 1
        sim.display && sleep_until(tm += sim.dt)
    end
    !sim.logging && push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
    #!sim.logging && push!(history, (a=a,))
    return history, r_total, iter, sim.rewardframes, sim.belframes
end

function simulate(sim::HIPPOSimulator{<:UnifiedPOMDP}; custom_reward=false)
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
    a = Union{Symbol, Vector{Int64}}[:nothing]
    info = nothing
    history = NamedTuple[]
    (sim.anim || sim.display) && (belframe = render(msim, (sp=s, bp=bp)))
    sim.display && display(belframe)
    sim.logging && push!(history, (s=s, a=a, sp=s, bp=bp, info=info))
    while !isterminal(msim, s) && iter < max_iter
        tm = time()
        #_, info = action_info(sim.planner, sim.b, tree_in_info = true)
        #tree = info[:tree] # maybe set POMCP option tree_in_info = true
        #a_traj = extract_trajectory(root(tree), 5)
        #a = first(a_traj)
        #try 
        a, _ = action_info(sim.planner, b)

        
        #catch e
            #a = :stay
        #    @warn "POMCP failed to find an action: ", e
        #    push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
        #    return history, r_total, iter
        #end
        remove_rewards(msim, s.robot) # remove reward at current state
        #display(msim.reward)
        if custom_reward
            sp, o = @gen(:sp, :o)(msim, s, a)
            r = sim_reward(msim, s, a, sp)
        else
            sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        end
        isterminal(msim, sp) && break
        
        msim.currentbatt = sp.battery

        r_total += d*r
        d *= discount_factor(msim)
        bp = update(sim.up, b, a, o)
        (sim.anim || sim.display) && (belframe = render(msim, (sp=sp, bp=bp)))
        (sim.anim || sim.display) && (rewardframe = render(msim, (sp=sp, bp=bp), true))
        #display(belframe)
        sim.display && display(belframe)
        sim.verbose && println(iter,"- | s: ", s.robot, " | orient: ", s.orientation, " | sbatt: ", s.battery, " | a: ", a, 
        " | sp_robot:", sp.robot, " | sp_target:", sp.target, " | spbatt: ", sp.battery, " | r:", r, " | o: ", o)
        #println(iter,"- | battery: ", sp.battery, " | dist_to_home: ", dist(sp.robot, msim.robot_init), " | s: ", sp.robot)
        sim.anim && push!(sim.rewardframes, rewardframe)
        sim.anim && push!(sim.belframes, belframe)
        sim.logging && push!(history, (s=s, a=a, sp=sp, o=o, r=r, bp=bp, info=info))
        #sim.logging && push!(history, (s=s,a=a))
        finalstate = s
        s = sp
        b = bp
        iter += 1
        sim.display && sleep_until(tm += sim.dt)
    end
    !sim.logging && push!(history, (s=finalstate, a=a, sp=sp, o=o, r=r, bp=b, info=info))
    #!sim.logging && push!(history, (a=a,))
    return history, r_total, iter, sim.rewardframes, sim.belframes
end

function predicted_path(sim::PachSimulator; pathlen::Int=10)
    (;msim,planner,sinit,b,up) = sim
    s = sinit

    _, info = action_info(planner, b, tree_in_info = true)
    tree = info[:tree] # maybe set POMCP option tree_in_info = true
    a_traj = extract_trajectory(root(tree), pathlen)
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