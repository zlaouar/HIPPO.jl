function targetsearch_observations(size)
    os = SVector{4,Int}[]
    for left in 0:size[1]-1
        for right in 0:size[1]-left-1
            for up in 0:size[2]-1
                for down in 0:size[2]-up-1
                    push!(os, SVector(left, right, up, down))
                end
            end
        end
    end
    return os
end

POMDPs.observations(m::TargetSearchPOMDP) = targetsearch_observations(m.size) 

function POMDPs.observation(m::TargetSearchPOMDP, a, sp)
    obs = [sp,]
    if norm(sp.robot-sp.target) == 1.0
        return SparseCat(obs, probs)
    elseif sp.robot == sp.target
    
    else

    end

end