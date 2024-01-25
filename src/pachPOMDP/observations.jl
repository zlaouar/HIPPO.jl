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


    probs = [0.9, 0.1]

    return SparseCat(OBSERVATIONS, probs)

end