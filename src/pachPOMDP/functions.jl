"""
    states (some of these functions are not used since the state space is too large)
"""
function POMDPs.stateindex(m::TargetSearchPOMDP{S,A,O}, s) where {S,A,O}
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

function POMDPs.states(m::TargetSearchPOMDP{S,A,O}) where {S,A,O}
    nonterm = vec(collect(FullState(SVector(c[1],c[2]), SVector(c[3],c[4]), BitVector(d)) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]) for d in Iterators.product(ntuple(s->[0,1],prod(m.size))...)))
    return push!(nonterm, FullState([-1,-1],[-1,-1],trues(prod(m.size))))
end

function POMDPs.initialstate(m::TargetSearchPOMDP{S,A,O}) where {S,A,O}
    return POMDPTools.Uniform(FullState(m.robot_init, SVector(x, y), trues(prod(m.size)), z) for x in 1:m.size[1], y in 1:m.size[2], z in 1:m.maxbatt)
end

function POMDPs.initialstate(m::RewardPOMDP)
    return POMDPTools.Uniform(RewardState(m.robot_init, SVector(x, y), trues(prod(m.size))) for x in 1:m.size[1], y in 1:m.size[2])
end

"""
    actions
"""
POMDPs.actions(m::TargetSearchPOMDP{S,A,O}) where {S,A,O} = (:left, :right, :up, :down, :stay, :nw, :ne, :sw, :se)

POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]

function bounce(m::TargetSearchPOMDP{S,A,O}, pos, offset) where {S,A,O}
    if clamp.(pos + offset, SVector(1,1), m.size) ∈ m.obstacles
        return pos
    end

    clamp.(pos + offset, SVector(1,1), m.size)
end

POMDPs.discount(m::TargetSearchPOMDP{S,A,O}) where {S,A,O} = 0.95 


function POMDPs.transition(m::TargetSearchPOMDP{S,A,O}, s, a) where {S,A,O}
    states = FullState[]
    probs = Float64[]
    remaining_prob = 1.0

    if isequal(s.robot, s.target)
        return Deterministic(FullState(SA[-1,-1], copy(s.target), s.visited, s.battery))
    end
    
    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, FullState(newrobot, s.target, copy(s.visited), s.battery-1))
    push!(probs, remaining_prob)

    for sp ∈ states
        sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
        sp.visited[sind] = 0
    end

    return SparseCat(states, probs)

end

function POMDPs.reward(m::TargetSearchPOMDP{S,A,O}, s::FullState, a::Symbol, sp::FullState) where {S,A,O}
    reward_running = -1.0
    reward_target = 0.0
    #reward_roi = 0.0

    if isequal(sp.robot, sp.target)# if target is found
        reward_running = 0.0
        reward_target = 1000.0 
        return reward_running + reward_target
    end
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end

    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        return reward_running + reward_target + m.reward[inds...]*s.visited[spind]
    end
end

function POMDPs.transition(m::RewardPOMDP, s, a)
    states = RewardState[]
    probs = Float64[]
    remaining_prob = 1.0

    if isequal(s.robot, s.target)
        return Deterministic(RewardState(SA[-1,-1], copy(s.target), s.visited))
    end
    
    if haskey(m.rois, s.robot)
        push!(states, RewardState(SA[-1,-1], SA[-1,-1], copy(s.visited))) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
    end


    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, RewardState(newrobot, s.target, copy(s.visited)))
    push!(probs, remaining_prob)


    for sp ∈ states
        sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
        sp.visited[sind] = 0
    end

    return SparseCat(states, probs)

end

function POMDPs.reward(m::RewardPOMDP, s::RewardState, a::Symbol, sp::RewardState)
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

    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        return reward_running + reward_target + reward_roi + m.reward[inds...]*s.visited[spind] # running cost
    end
end

set_default_graphic_size(18cm,14cm)

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP{S,A,O}, step) where {S,A,O}
    #set_default_graphic_size(14cm,14cm)
    nx, ny = m.size
    cells = []
    target_marginal = zeros(nx, ny)

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
        max_op = maximum(norm_top)
        min_op = minimum(norm_top)
        frac = (opval-min_op)/(max_op-min_op)
        clr = get(ColorSchemes.bamako, frac)
        


        target = compose(context(), rectangle(), fill(clr), stroke("gray"))
        #println("opval: ", t_op)
        compose!(cell, target)

        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.00000001mm), cells...)
    outline = compose(context(), linewidth(0.01mm), rectangle(), fill("white"), stroke("black"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("blue"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, star(0.5,0.5,0.8,5,0.5), fill("orange"), stroke("black"))
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

    return inds
end


function POMDPTools.ModelTools.render(m::TargetSearchPOMDP{S,A,O}, step, plt_reward::Bool) where {S,A,O}
    nx, ny = m.size
    cells = []
    
    minr = minimum(m.reward)-1
    maxr = maximum(m.reward)
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        r = m.reward[rewardinds(m, SA[x,y])...]
        if iszero(r)
            target = compose(context(), rectangle(), fill("black"), stroke("gray"))
        else
            frac = (r-minr)/(maxr-minr)
            clr = get(ColorSchemes.turbo, frac)
            target = compose(context(), rectangle(), fill(clr), stroke("gray"), fillopacity(0.5))
        end

        compose!(cell, target)
        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.00000001mm), cells...)
    outline = compose(context(), linewidth(0.01mm), rectangle(), fill("white"), stroke("black"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("blue"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, star(0.5,0.5,1.0,5,0.5), fill("orange"), stroke("black"))
    else
        robot = nothing
        target = nothing
    end
    sz = min(w,h)
    #return compose(context((w-sz)/2, (h-sz)/2, sz, (ny/nx)*sz), robot, target, grid, outline)
    return compose(context((w-sz)/2, (h-sz)/2, sz, sz), robot, target, grid, outline)
end

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP{S,A,O}, step, points::Vector{Vector{Int}}) where {S,A,O}
    nx, ny = m.size
    cells = []
    minr = minimum(m.reward)-1
    maxr = maximum(m.reward)
    iter = 1
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        r = m.reward[rewardinds(m, SA[x,y])...]
        if iszero(r)
            target = compose(context(), rectangle(), fill("black"), fillopacity(0.9), stroke("gray"))
            compose!(cell, target)
        else
            frac = (r-minr)/(maxr-minr)
            clr = get(ColorSchemes.turbo, frac)
            target = compose(context(), rectangle(), fill(clr), stroke("gray"), fillopacity(0.5))
            # target = compose(context(), rectangle(), fillopacity(normie(m.reward[rewardinds(m,SA[x,y])...],m.reward)), fill("green"), stroke("gray"))
        end
        if [x,y] ∈ points
            roi = compose(context(), circle(), fill("green2"), stroke("green2"), linewidth(0.5mm))
            compose!(cell, roi)
        else
            compose!(cell, target)
        end

        push!(cells, cell)
        iter += 1
    end
    grid = compose(context(), linewidth(0.00000001mm), cells...)
    outline = compose(context(), linewidth(0.01mm), rectangle(), fill("white"), stroke("black"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("blue"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, star(0.5,0.5,1.8,5,0.5), fill("orange"), stroke("black"))
    else
        robot = nothing
        target = nothing
    end
    sz = min(w,h)
    return compose(context((w-sz)/2, (h-sz)/2, sz, (ny/nx)*sz), robot, target, grid, outline)
end

function dist(curr, start)
    sum(abs.(curr-start))
end

function POMDPs.isterminal(m::TargetSearchPOMDP{S,A,O}, s::FullState) where {S,A,O}
    required_batt = dist(s.robot, m.robot_init)
    return s.battery - required_batt <= 1 || s.robot == SA[-1,-1] 
end

POMDPs.isterminal(m::RewardPOMDP, s::TSState) = s.robot == SA[-1,-1]

POMDPs.isterminal(m::TargetSearchPOMDP{S,A,O}, s::TSState, target) where {S,A,O} = s.robot == target