function rewardinds(m, s::TSState)
    correct_ind = reverse(s.robot)
    xind = m.size[1]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
end

function remove_rewards(pomdp, s)
    pomdp.reward[rewardinds(pomdp,s)...] = 0.0
end

sleep_until(t) = sleep(max(t-time(), 0.0))

function generatelocation(atraj, robotloc)
    loctraj = Vector{Vector{Float64}}(undef, length(atraj))
    s
    for i in eachindex(loctraj)
        if atraj[i] == "left"
            
        else if atraj[i] == "left"
        loctraj[i] = s.robot
    end
end

function predicted_path(msim::TargetSearchPOMDP, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    d = 1.0

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

    rewardframe = render(msim, (sp=sp, bp=b), true)
    display(rewardframe)
    
    s = sp

    return a_traj
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
        msim.reward[rewardinds(msim,s)...] = 0.0 # remove reward at current state
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