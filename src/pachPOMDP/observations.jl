"""
    observations(m::TargetSearchPOMDP)

Retrieve observations in TargetSearch observation space

The the observations are ordered as follows:
    1: The next waypoint is reached
    2: FALCO gather_action is executed
"""


POMDPs.observations(m::TargetSearchPOMDP) = OBSERVATIONS #vec(collect(BitVector([c[1],c[2],c[3],c[4],c[5]]) for c in Iterators.product(0:1, 0:1, 0:1, 0:1, 0:1)))
POMDPs.obsindex(m::TargetSearchPOMDP, o::Symbol) = obsind[o]

function POMDPs.observation(m::TargetSearchPOMDP, a::Symbol, sp::TSState)
    if norm(sp.robot-sp.target) <= 2.0
        if sp.robot[1]-sp.target[1] >= 2.0 && a == :left 
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif sp.robot[1]-sp.target[1] <= 2.0 && a == :right
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif sp.robot[2]-sp.target[2] >= 2.0 && a == :down
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif sp.robot[2]-sp.target[2] <= 2.0 && a == :up
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :sw
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :se
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :nw
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :ne
            return SparseCat(OBSERVATIONS, [0.2, 0.8])
        else

            return SparseCat(OBSERVATIONS, [0.95, 0.05])
        end
    else
        return SparseCat(OBSERVATIONS, [0.95, 0.05])
    end




    # if norm(sp.robot-sp.target) <= 2.0 
    #     probs = [0.2, 0.8]
    # else
    #     probs = [0.9, 0.1]
    # end

    #probs = [0.9, 0.1]
    return SparseCat(OBSERVATIONS, probs)

end