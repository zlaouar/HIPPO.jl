function POMDPs.states(m::TSPOMDPBattery) 
    nonterm = vec(collect(TSStateBattery(SVector(c[1],c[2]), SVector(c[3],c[4])) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2])))
    return push!(nonterm, TSStateBattery([-1,-1],[-1,-1]))
end

POMDPs.actions(m::TSPOMDPBattery) = (:left, :right, :up, :down, :stay)

POMDPs.discount(m::TSPOMDPBattery) = 0.95

function POMDPs.stateindex(m::TSPOMDPBattery, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

POMDPs.actionindex(m::TSPOMDPBattery, a) = actionind[a]


function bounce(m::TSPOMDPBattery, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

function POMDPs.transition(m::TSPOMDPBattery, s, a)
    states = TSStateBattery[]
    probs = Float64[]
    remaining_prob = 1.0

    #if isequal(s.robot, s.target)
    #    return Deterministic(TSStateBattery(SA[-1,-1], SA[-1,-1], s.battery))
    #end

    if haskey(m.rois, s.robot)
        push!(states, TSStateBattery(SA[-1,-1], SA[-1,-1])) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
    end


    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, TSStateBattery(newrobot, s.target, s.battery-1))
    push!(probs, remaining_prob)


    return SparseCat(states, probs)

end

function POMDPs.reward(m::TSPOMDPBattery, s::TSStateBattery, a::Symbol, sp::TSStateBattery)
    reward_running = -1.0
    reward_target = 0.0
    reward_roi = 0.0
    #cost_batt = 0.0

    if sp.robot == sp.target && sp.robot != SA[-1,-1]# if target is found
        reward_running = 0.0
        reward_target = 100.0 
    end
    if sp.robot == SA[-1,-1]
        reward_running = 0.0
        reward_roi = 0.0
    end
    #= if sp.battery <= 0 && sp.robot != m.robot_init
        cost_batt = -10_000.0
    end =#
        
    return reward_running + reward_target + reward_roi #+ cost_batt
end

function POMDPs.initialstate(m::TSPOMDPBattery)
    #return Deterministic(TSStateBattery(m.robot_init,m.targetloc,m.maxbatt))
    return POMDPTools.Uniform(TSStateBattery(m.robot_init, SVector(x, y), z) for x in 1:m.size[1], y in 1:m.size[2], z in 1:m.maxbatt)
end

function POMDPTools.ModelTools.render(m::TSPOMDPBattery, step)
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


#POMDPs.isterminal(m::TSPOMDPBattery, s::TSStateBattery) = s.robot == SA[-1,-1]
function dist(curr, start)
    sum(abs.(curr-start))
end

function POMDPs.isterminal(m::TSPOMDPBattery, s::TSStateBattery)
    required_batt = dist(s.robot, m.robot_init)
    return required_batt == s.battery 
end