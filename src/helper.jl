function svgstogif(frames)
    for (i,frame) in enumerate(frames1)
        imgstr = "$i"
        imgstr = lpad(imgstr, 2, '0') * ".svg"
        draw(SVG(joinpath(@__DIR__,"../tmp/img/","$imgstr")), frame)
    end
end

function BasicPOMCP.estimate_value(estimator::Union{BasicPOMCP.SolvedPORollout,BasicPOMCP.SolvedFORollout}, pomdp::UnifiedPOMDP, start_state, h::BeliefNode, steps::Int)
    effective_horizon = pomdp.maxbatt-start_state.battery + 1
    steps_to_go = pomdp.rollout_depth - effective_horizon
    BasicPOMCP.rollout(estimator, pomdp, start_state, h, steps_to_go)
end

struct FixedPolicy <: Function end

(::FixedPolicy)(s) = :up

function statedir(pos1, pos2)
    if (pos1[1]-pos2[1]) >= 1 # pos2 left of robot
        return :left
    elseif (pos1[1]-pos2[1]) <= -1 # pos2 right of robot
        return :right
    elseif (pos1[2]-pos2[2]) >= 1 # pos2 below robot
        return :down
    elseif (pos1[2]-pos2[2]) <= -1 # pos2 above robot
        return :up
    else
        return :stay
    end
end

function cell_list(pos1, pos2)
    pos1 = collect(pos1)
    list = []
    while pos1 != pos2
        if pos1[1] < pos2[1]
            pos1[1] += 1
        elseif pos1[1] > pos2[1]
            pos1[1] -= 1
        end
        if pos1[2] < pos2[2]
            pos1[2] += 1
        elseif pos1[2] > pos2[2]
            pos1[2] -= 1
        end
        push!(list, copy(pos1))
    end    
    return list
end

function primitive_actions_from_macro(current_pos, macro_action)
    actions = []
    cells = cell_list(current_pos, macro_action)
    push!(actions, orientdir[cells[1] - current_pos])
    for i ∈ 1:length(cells)-1
        push!(actions, orientdir[cells[i+1] - cells[i]])
    end
    return actions
end

struct TargetSearchMDPPolicy{P} <: Policy
    vi_policy::P
end

struct GreedyPolicy{P} <: Policy
    pomdp::P
end

function POMDPs.action(p::GreedyPolicy, s)
    if rand() < 1.0
        return statedir(s.robot, s.target)
    else
        return statedir(s.robot, mat_to_inertial_inds(p.pomdp.size, Tuple(argmax(p.pomdp.reward))))
    end
    # return statedir(s.robot, s.target)
end

function POMDPs.action(p::TargetSearchMDPPolicy, s)
    newS = BasicState(s.robot, s.target)
    return DiscreteValueIteration.action(p.vi_policy, newS)
end

function AbstractTrees.children(t::BasicPOMCP.POMCPObsNode)
    return t.tree.children[t.node]
end

root(tree::BasicPOMCP.POMCPTree) = BasicPOMCP.POMCPObsNode(tree, 1)

function extract_trajectory(node::BasicPOMCP.POMCPObsNode, depth)
    #children
    a_traj = Vector{Symbol}(undef, depth)

    t = node.tree
    lenb = length(t.total_n)
    lenba = length(t.n)
    len = lenb + lenba
    children = Vector{Vector{Int}}(undef, len)
    ba_children = [Set{Int}() for i in 1:lenba]
    for (ha_o, c) in t.o_lookup
        ha, o = ha_o
        push!(ba_children[ha], c)
    end

    for b in 1:lenb
        children[b] = t.children[b] .+ lenb
    end

    for ba in 1:lenba
        children[ba+lenb] = collect(ba_children[ba])
    end

    oind = 1
    d = t.o_lookup
    for i in 1:depth
        # get all action indices from current observation node
        ainds = children[oind] .- lenb
        an = t.n[ainds]
        avals = t.v[ainds]

        # TODO: break visit ties with value
        _, aind = findmax(an) # find action with highest visit count
        aind = ainds[aind]
        a_traj[i] = t.a_labels[aind]

        # get all observation indices that have the action as a parent
        inds = findall(x-> x==aind, getindex.(keys(d), 1))
        oinds = collect(values(d))[inds]
        try
            oind = oinds[findmax(t.total_n[oinds])[2]] # get observation node with highest visit count
        catch e
            @warn "error in extract_trajectory"
            return a_traj[1:i]
            #inchrome(D3Tree(t))
            #error("no null observation")
        end

        #oinds = [t.o_lookup[(ainds[i], [0,0,0,0,0])] for i in eachindex(ainds)]

        #=inds = findall(x-> x==onull, getindex.(keys(d), 2))
        ainds = getindex.(keys(d), 1)[inds]
        aind = ainds[findmax(t.v[ainds])[2]] =#
        # get observation from that action node

        #= avals = t.v[children[oind] .- lenb]
        ainfo = findmax(avals)
        aind = ainfo[2]
        a_traj[i] = t.a_labels[aind]
        println("aind: ", aind, " | avals: ", avals, " | oind: ", oind)
        println(t.o_lookup)
        try
            oind = t.o_lookup[(aind,[0,0,0,0,0])]
        catch e
            inchrome(D3Tree(t))
            error("no null observation")
        end
        println("____________________________________________________________") =#
        
    end
    return a_traj
end

function next_action(children, aind, o)
    # get index of observed node from current action node
    inds = findall(x-> x==aind, getindex.(keys(t.o_lookup), 1))
    oinds = collect(values(d))[inds]

    recieved_observation_ind = oinds[findfirst(obs -> obs == o, t.o_labels[oinds])]

    # find best action to take from observation node
    ainds = children[oind] .- lenb
    avals = t.v[ainds]
    _, aind = findmax(avals) # find action with highest value
    aind = ainds[aind]
    
    return t.a_labels[aind]
end

function π_conditional(node::BasicPOMCP.POMCPObsNode)
    t = node.tree
    lenb = length(t.total_n)
    lenba = length(t.n)
    len = lenb + lenba
    children = Vector{Vector{Int}}(undef, len)
    ba_children = [Set{Int}() for i in 1:lenba]
    for (ha_o, c) in t.o_lookup
        ha, o = ha_o
        push!(ba_children[ha], c)
    end

    for b in 1:lenb
        children[b] = t.children[b] .+ lenb
    end

    for ba in 1:lenba
        children[ba+lenb] = collect(ba_children[ba])
    end
    
    return children
end

function next_action(hnode::BasicPOMCP.POMCPObsNode, aprev)
    t = hnode.tree
    h = hnode.node

    # Descend tree according to action taken at previous wp and observation received at next wp
    
    # what is the action node index that the agent took at the previous wp
    ha = findfirst(a -> a == aprev, t.a_labels[t.children[h]])
    # @info ha

    # descend down tree from "waypoint reached" observation node
    hao = get(t.o_lookup, (ha, :next_waypoint), 0)
    # @info hao

    if hao == 0
        @warn "wp reached observation not in tree"
        #inchrome(D3Tree(t))
        return :stay
    end

    # find best child action to take from wp reached observation 
    ainds = t.children[hao]
    # avals = t.v[ainds]
    avals = t.n[ainds]
    _, ind = findmax(avals) # find action with highest value
    # @info ind
    haoa = ainds[ind]
    
    return t.a_labels[haoa]
end