function POMDPs.states(m::TargetSearchPOMDP) 
    nonterm = vec(collect(TSState(SVector(c[1],c[2]), SVector(c[3],c[4]), BitVector(d)) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]) for d in Iterators.product(ntuple(s->[0,1],prod(m.size))...)))
    return push!(nonterm, TSState([-1,-1],[-1,-1],trues(prod(m.size))))
end

POMDPs.actions(m::TargetSearchPOMDP) = (:left, :right, :up, :down, :stay)

POMDPs.discount(m::TargetSearchPOMDP) = 0.95

function POMDPs.stateindex(m::TargetSearchPOMDP, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]

function bounce(m::TargetSearchPOMDP, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

function POMDPs.transition(m::TargetSearchPOMDP, s, a)
    states = TSState[]
    probs = Float64[]
    remaining_prob = 1.0

    if isequal(s.robot, s.target)
        return Deterministic(TSState(SA[-1,-1], copy(s.target), s.visited))
    end
    
    if haskey(m.rois, s.robot)
        push!(states, TSState(SA[-1,-1], SA[-1,-1], copy(s.visited))) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
    end


    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, TSState(newrobot, s.target, copy(s.visited)))
    push!(probs, remaining_prob)


    for sp ∈ states
        sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
        sp.visited[sind] = 0
    end

    return SparseCat(states, probs)

end

function POMDPs.reward(m::TargetSearchPOMDP, s::TSState, a::Symbol, sp::TSState)
    reward_running = -1.0
    reward_target = 0.0
    reward_roi = 0.0

    if isequal(sp.robot, sp.target)# if target is found
        reward_running = 0.0
        reward_target = 100.0 
        return reward_running + reward_target + reward_roi
    end
    if isterminal(m, sp)
        return 0.0
    end

    #correct_ind = reverse(sp.robot)
    #xind = m.size[2]+1 - correct_ind[1]
    #inds = [xind, correct_ind[2]]
    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]

        return reward_running + reward_target + reward_roi + m.reward[inds...]*s.visited[spind] # running cost
        #try 
        #    return reward_running + reward_target + reward_roi + m.reward[inds...]*s.visited[spind] # running cost
        #catch
        #    @error("inds: $inds, | robot: $(sp.robot)")
        #end
    end
end

function POMDPs.initialstate(m::TargetSearchPOMDP)
    return POMDPTools.Uniform(TSState(m.robot_init, SVector(x, y), trues(prod(m.size))) for x in 1:m.size[1], y in 1:m.size[2])
end

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, step)
    #set_default_graphic_size(14cm,14cm)
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
    #display(target_marginal)
    norm_top = normalize(target_marginal)
    #display(norm_top)
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        t_op = norm_top[x,y]
        # TO-DO Fix This
        if t_op > 1.0
            if t_op < 1.001
                t_op = 0.999
            else
                @error("t_op > 1.001", t_op)
            end
        end
        opval = t_op
        if opval > 0.0 
            opval = clamp(t_op*2,0.05,1.0)
        end

        target = compose(context(), rectangle(), fillopacity(opval), fill("orange"), stroke("gray"))
        if [x,y] in rois
            roi = compose(context(), rectangle(), fill("transparent"), stroke("white"), linewidth(1.2mm))
            compose!(cell, target, roi)
        else
            compose!(cell, target)
        end

        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), fill("white"), stroke("gray"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("green"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, circle(0.5, 0.5, 0.5), fill("orange"))
    else
        robot = nothing
        target = nothing
    end 
    #img = read(joinpath(@__DIR__,"../..","drone.png"));
    #robot = compose(robot_ctx, bitmap("image/png",img, 0, 0, 1, 1))
    #person = read(joinpath(@__DIR__,"../..","missingperson.png"));
    #target = compose(target_ctx, bitmap("image/png",person, 0, 0, 1, 1))

    sz = min(w,h)
    
    return compose(context((w-sz)/2, (h-sz)/2, sz, sz), robot, target, grid, outline)
end

function normie(input, a)
    return (input-minimum(a))/(maximum(a)-minimum(a))
end

function rewardinds(m, pos::SVector{2, Int64})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    #= try 
        m.reward[inds...]
    catch
        @error("inds: $inds, | robot: $pos")
    end =#
    return inds
end

set_default_graphic_size(18cm,14cm)

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, step, plt_reward::Bool)
    nx, ny = m.size
    cells = []
    rois = collect(keys(m.rois))
    
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        if iszero(m.reward[rewardinds(m, SA[x,y])...])
            target = compose(context(), rectangle(), fill("black"), stroke("gray"))
        else
            target = compose(context(), rectangle(), fillopacity(normie(m.reward[rewardinds(m,SA[x,y])...],m.reward)), fill("red"), stroke("gray"))
        end
        if [x,y] in rois
            roi = compose(context(), rectangle(), fill("transparent"), stroke("white"), linewidth(1.2mm))
            compose!(cell, target, roi)
        else
            compose!(cell, target)
        end

        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), fill("white"), stroke("gray"))

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

POMDPs.isterminal(m::TargetSearchPOMDP, s::TSState) = s.robot == SA[-1,-1]