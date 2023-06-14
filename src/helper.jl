function svgstogif(frames)
    for (i,frame) in enumerate(frames1)
        imgstr = "$i"
        imgstr = lpad(imgstr, 2, '0') * ".svg"
        draw(SVG(joinpath(@__DIR__,"../tmp/img/","$imgstr")), frame)
    end
end

struct FixedPolicy <: Function end

(::FixedPolicy)(s) = :up

struct TargetSearchMDPPolicy{P} <: Policy
    vi_policy::P
end

function POMDPs.action(p::TargetSearchMDPPolicy, s)
    newS = TSStateBasic(s.robot, s.target)
    return action(p.vi_policy, newS)
end

function AbstractTrees.children(t::BasicPOMCP.POMCPObsNode)
    #
    

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
    for i in 1:depth
        avals = t.v[children[oind] .- lenb]
        aind = findmax(avals)[2]
        a_traj[i] = t.a_labels[aind]
        println("aind: ", aind, " | oind: ", oind)
        oind = t.o_lookup[(aind,[0,0,0,0,0])]
        
    end
    return a_traj
end
