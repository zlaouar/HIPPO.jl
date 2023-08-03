"""
    observations(m::TargetSearchPOMDP)

Retrieve observations in TargetSearch observation space

The the observations are ordered as follows:
    1: The target is not observed
    2: The target is in same grid cell as robot
    3: The target is to the left of the robot
    4: The target is to the right of the robot
    5: The target is below the robot
    6: The target is above the robot
"""
POMDPs.observations(m::TargetSearchPOMDP) = OBSERVATIONS #vec(collect(BitVector([c[1],c[2],c[3],c[4],c[5]]) for c in Iterators.product(0:1, 0:1, 0:1, 0:1, 0:1)))
POMDPs.obsindex(m::TargetSearchPOMDP, o::BitVector) = obsind[o]

function POMDPs.observation(m::TargetSearchPOMDP, a::Symbol, sp::TSState)
    #obs = [BitVector([0,0,0,0,0]), BitVector([1,0,0,0,0]), BitVector([0,1,0,0,0]), BitVector([0,0,1,0,0]), BitVector([0,0,0,1,0]), BitVector([0,0,0,0,1])]

    if norm(sp.robot-sp.target) == 1.0 # target and robot within one grid cell of each other 
        targetloc = targetdir(sp)

        if targetloc == :left
            probs = [0.0, 0.0, 0.50, 0.0, 0.25, 0.25]
        elseif targetloc == :right
            probs = [0.0, 0.0, 0.0, 0.50, 0.25, 0.25]
        elseif targetloc == :up
            probs = [0.0, 0.0, 0.25, 0.25, 0.0, 0.50]
        elseif targetloc == :down
            probs = [0.0, 0.0, 0.25, 0.25, 0.50, 0.0]
        end

        return SparseCat(OBSERVATIONS, probs)
    end

    if sp.robot == sp.target # target and robot in same grid cell
        probs = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
        return SparseCat(OBSERVATIONS, probs)
    end

    probs = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return SparseCat(OBSERVATIONS, probs)

end