function rewardinds(m, s::TSStateBattery)
    correct_ind = reverse(s.robot)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
end

sleep_until(t) = sleep(max(t-time(), 0.0))

function customsim(msolve::TSPOMDPBattery, msim::TSPOMDPBattery, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 10
    dt = 1/max_fps
    d = 1.0
    sim_states = TSStateBattery[]
    belframes = Frames(MIME("image/png"), fps=10)
    while !isterminal(msim, s) && iter < 500
        tm = time()
        a = action(planner, b)
        sp, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        belframe = render(msim, (sp=sp, bp=b))
        display(belframe)
        println(s)
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
    return r_total, sim_states, belframes
end