function svgstogif(frames)
    for (i,frame) in enumerate(frames1)
        imgstr = "$i"
        imgstr = lpad(imgstr, 2, '0') * ".svg"
        draw(SVG(joinpath(@__DIR__,"../tmp/img/","$imgstr")), frame)
    end
end

sleep_until(t) = sleep(max(t-time(), 0.0))

function rewardinds(m, s)
    correct_ind = reverse(s.robot)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
end

function customsim(msolve::TargetSearchPOMDP, msim::TargetSearchPOMDP, planner, up, b, sinit)
    r_total = 0.0
    s = sinit
    o = Nothing
    iter = 0
    max_fps = 3
    dt = 1/max_fps
    d = 1.0
    sim_states = TSState[]
    frames1 = []
    #frames1 = Frames(MIME("image/png"), fps=4)
    #frames2 = Frames(MIME("image/png"), fps=4)
    while !isterminal(msim, s) && iter < 500
    #for _ in 1:500
        tm = time()
        a = action(planner, b)
        msim.reward[rewardinds(msim,s)...] = 0.0 # remove reward at current state
        print(rewardinds(msim,s))
        s, o, r = @gen(:sp,:o,:r)(msim, s, a)
        r_total += d*r
        d *= discount(msim)
        b = update(up, b, a, o)
        tmp = render(msim, (sp=s, bp=b), true)
        display(tmp)
        sleep_until(tm += dt)
        iter += 1
        println(iter)
        if iter > 1000
            roi_states = [[1,9],[1,10],[1,8]]
            probs = [0.8,0.8,0.8]
            roi_points = Dict(roi_states .=> probs)
            msolve.rois = roi_points
            planner = solve(solver, msolve)
        end
        push!(sim_states, s)
        push!(frames1, tmp)
        #push!(frames2, render(msim, (sp=s, bp=b), true))
        #if isterminal(msim, s)
        #    break
        #end
    end
    return s, r_total, sim_states, frames1
end