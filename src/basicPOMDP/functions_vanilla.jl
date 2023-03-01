function POMDPs.states(m::TSPOMDPBasic) 
    nonterm = vec(collect(TSStateBasic(SVector(c[1],c[2]), SVector(c[3],c[4])) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2])))
    return push!(nonterm, TSStateBasic([-1,-1],[-1,-1]))
end

POMDPs.actions(m::TSPOMDPBasic) = (:left, :right, :up, :down, :stay)

POMDPs.discount(m::TSPOMDPBasic) = 0.95

function POMDPs.stateindex(m::TSPOMDPBasic, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

POMDPs.actionindex(m::TSPOMDPBasic, a) = actionind[a]


function bounce(m::TSPOMDPBasic, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

function POMDPs.transition(m::TSPOMDPBasic, s, a)
    states = TSStateBasic[]
    probs = Float64[]
    remaining_prob = 1.0

    if haskey(m.rois, s.robot)
        push!(states, TSStateBasic(SA[-1,-1], SA[-1,-1])) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
    end


    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, TSStateBasic(newrobot, s.target))
    push!(probs, remaining_prob)


    return SparseCat(states, probs)

end

function POMDPs.reward(m::TSPOMDPBasic, s::TSStateBasic, a::Symbol, sp::TSStateBasic)
    rmat = deepcopy(m.reward)
    #rmat = m.reward 
    correct_ind = reverse(s.robot)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    #m.reward[inds...] = 0.0

    reward_running = -1.0
    reward_target = 0.0
    reward_roi = 0.0
    if sp.robot == sp.target && sp.robot != SA[-1,-1]# if target is found
        reward_running = 0.0
        reward_target = 100.0 
    end
    if sp.robot == SA[-1,-1]
        reward_running = 0.0
        reward_roi = 0.0
    end
    #m.reward[inds...] = 0.0
    if !isempty(rmat)
        #display(rmat[inds...])
        return reward_running + reward_target + reward_roi + rmat[inds...] # running cost
    else
        reward_running + reward_target + reward_roi
    end
end

function POMDPs.initialstate(m::TSPOMDPBasic)
    return POMDPTools.Uniform(TSStateBasic(m.robot_init, SVector(x, y)) for x in 1:m.size[1], y in 1:m.size[2])
end

function POMDPTools.ModelTools.render(m::TSPOMDPBasic, step)
    nx, ny = m.size
    cells = []
    target_marginal = zeros(nx, ny)
    rois = collect(keys(m.rois))

    if haskey(step, :bp) && !ismissing(step[:bp])
        for sp in support(step[:bp])
            p = pdf(step[:bp], sp)
            if sp.target != [-1,-1] # TO-DO Fix this
                target_marginal[sp.target...] += p
            end
        end
    end
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        t_op = sqrt(target_marginal[x,y])
        # TO-DO Fix This
        if t_op > 1.0
            t_op = 0.999
        end
        
        target = compose(context(), rectangle(), fillopacity(t_op), fill("yellow"), stroke("gray"))
        if [x,y] in rois
            roi = compose(context(), rectangle(), fill("transparent"), stroke("white"), linewidth(1.2mm))
            compose!(cell, target, roi)
        else
            compose!(cell, target)
        end

        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), fill("black"), stroke("gray"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("green"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, circle(0.5, 0.5, 0.5), fill("orange"))
    else
        robot = nothing
        target = nothing
    end
    sz = min(w,h)
    return compose(context((w-sz)/2, (h-sz)/2, sz, sz), robot, target, grid, outline)
end


POMDPs.isterminal(m::TSPOMDPBasic, s::TSStateBasic) = s.robot == SA[-1,-1]