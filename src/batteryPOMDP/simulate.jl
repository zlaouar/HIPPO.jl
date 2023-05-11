function rewardinds(m, s::TSStateBattery)
    correct_ind = reverse(s.robot)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
end

sleep_until(t) = sleep(max(t-time(), 0.0))

#= struct SimHist
    x::Matrix{Float64}
end =#

function getaction(state::SVector{2, Int}, goal::SVector{2, Int})
    if state[1] > goal[1]
        return :left
    elseif state[2] < goal[2]
        return :up
    elseif state[1] < goal[1] 
        return :right
    elseif state[2] > goal[2] 
        return :down
    else
        return :stay
    end
end

function returnToLand!(msim, sp, up, b, r_total, sim_states, belframes, d, dt)
    s = sp
    while sp.robot != msim.robot_init
        tm = time()
        a = getaction(sp.robot, msim.robot_init)
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        println("state: ", s)
        println("action: ", a)
        println("obs: ", o)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        belframe = render(msim, (sp=sp, bp=b))
        display(belframe)
        #println("tm: ", tm, " dt: ", dt)
        sleep_until(tm += dt)
        push!(sim_states, sp)
        push!(belframes, belframe)

        s = sp
        
    end
end

function customsim(msolve::TSPOMDPBattery, msim::TSPOMDPBattery, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 7
    dt = 1/max_fps
    d = 1.0
    sim_states = TSStateBattery[]
    belframes = Frames(MIME("image/png"), fps=10)
    sp = sinit
    while !isterminal(msim, s) && iter < 500 && s.battery != 0
        tm = time()
        a = action(planner, b)
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        println("_____________________________")
        println("state: ", s)
        println("action: ", a)
        #println("reward: ", r)
        println("obs: ", o)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        belframe = render(msim, (sp=sp, bp=b))
        display(belframe)
        sleep_until(tm += dt)
        iter += 1
        #println(iter,"- | s: ", s, " | sp:", sp, " | r:", r, " | o: ", o)
        push!(sim_states, sp)
        push!(belframes, belframe)

        s = sp
        #push!(frames2, render(msim, (sp=s, bp=b), true))
        #if isterminal(msim, s)
        #    break
        #end
    end
    println("current battery: ", s.battery, " - required battery: ", dist(s.robot, msim.robot_init))

    returnToLand!(msim, sp, up, b, r_total, sim_states, belframes, d, dt)
    return r_total, sim_states, belframes
end