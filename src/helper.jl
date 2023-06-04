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
    return t.tree.children[t.node]
end

