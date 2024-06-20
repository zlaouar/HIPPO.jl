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


# function POMDPs.transition(m::TargetSearchPOMDP{S,A,O}, s, a) where {S,A,O}
#     states = FullState[]
#     probs = Float64[]
#     remaining_prob = 1.0

#     if isequal(s.robot, s.target)
#         return Deterministic(FullState(SA[-1,-1], copy(s.target), s.visited, s.battery))
#     end
    
#     newrobot = bounce(m, s.robot, actiondir[a])

#     push!(states, FullState(newrobot, s.target, copy(s.visited), s.battery-(m.resolution/25)))
#     push!(probs, remaining_prob)

#     for sp ∈ states
#         sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
#         sp.visited[sind] = 0
#     end

#     return SparseCat(states, probs)

# end

function projected_states(m, s, a)
    if s.robot[1]-s.target[1] >= 2.0 && a == :left 
        states = [bounce(m, s.robot, SVector(-1,0)), 
                  bounce(m, s.robot, SVector(-2,0))]
    elseif s.robot[1]-s.target[1] <= 2.0 && a == :right
        states = [bounce(m, s.robot, SVector(1,0)), 
                  bounce(m, s.robot, SVector(2,0))]
    elseif s.robot[2]-s.target[2] >= 2.0 && a == :down
        states = [bounce(m, s.robot, SVector(0,-1)), 
                  bounce(m, s.robot, SVector(0,-2))]
    elseif s.robot[2]-s.target[2] <= 2.0 && a == :up
        states = [bounce(m, s.robot, SVector(0,1)), 
                  bounce(m, s.robot, SVector(0,2))]
    elseif (s.robot[1]-s.target[1] >= 2.0) && (s.robot[2]-s.target[2] >= 2.0) && a == :sw
        states = [bounce(m, s.robot, SVector(-1,-1)), 
                  bounce(m, s.robot, SVector(-2,-2))]
    elseif (s.robot[1]-s.target[1] <= 2.0) && (s.robot[2]-s.target[2] >= 2.0) && a == :se
        states = [bounce(m, s.robot, SVector(1,-1)), 
                  bounce(m, s.robot, SVector(2,-2))]
    elseif (s.robot[1]-s.target[1] >= 2.0) && (s.robot[2]-s.target[2] <= 2.0) && a == :nw
        states = [bounce(m, s.robot, SVector(-1,1)), 
                  bounce(m, s.robot, SVector(-2,2))]
    elseif (s.robot[1]-s.target[1] <= 2.0) && (s.robot[2]-s.target[2] <= 2.0) && a == :ne
        states = [bounce(m, s.robot, SVector(1,1)), 
                  bounce(m, s.robot, SVector(2,2))]
    else
        states = []
    end


    # elseif s.robot[1]-s.target[1] <= 2.0 && a == :right
    #     states = [s.robot + SVector(1,0), s.robot + SVector(2,0)]
    # elseif s.robot[2]-s.target[2] >= 2.0 && a == :down
    #     states = [s.robot + SVector(0,-1), s.robot + SVector(0,-2)]
    # elseif s.robot[2]-s.target[2] <= 2.0 && a == :up
    #     states = [s.robot + SVector(0,1), s.robot + SVector(0,2)]
    # elseif (s.robot[1]-s.target[1] >= 2.0) && (s.robot[2]-s.target[2] >= 2.0) && a == :sw
    #     states = [s.robot + SVector(-1,-1), s.robot + SVector(-2,-2)]
    # elseif (s.robot[1]-s.target[1] <= 2.0) && (s.robot[2]-s.target[2] >= 2.0) && a == :se
    #     states = [s.robot + SVector(1,-1), s.robot + SVector(2,-2)]
    # elseif (s.robot[1]-s.target[1] >= 2.0) && (s.robot[2]-s.target[2] <= 2.0) && a == :nw
    #     states = [s.robot + SVector(-1,1), s.robot + SVector(-2,2)]
    # elseif (s.robot[1]-s.target[1] <= 2.0) && (s.robot[2]-s.target[2] <= 2.0) && a == :ne
    #     states = [s.robot + SVector(1,1), s.robot + SVector(2,2)]
    # else
    #     states = []
    # end
end

function POMDPs.transition(m::TargetSearchPOMDP{S,A,O}, s, a) where {S,A,O}
    states = FullState[]
    probs = Float64[]
    gather_info_prob = 0.2
    wp_reached_prob = 0.8
    gather_info_time = 2*(m.resolution/25)

    if isequal(s.robot, s.target)
        return Deterministic(FullState(SA[-1,-1], copy(s.target), s.visited, s.battery))
    end
    
    newrobot = bounce(m, s.robot, actiondir[a])

    # drone reached next waypoint
    push!(states, FullState(newrobot, s.target, copy(s.visited), s.battery-(m.resolution/25)))

    # FALCO gather_action is executed
    if norm(s.robot-s.target) <= 2.0*sqrt(2.0)
        push!(probs, wp_reached_prob)
        locs = projected_states(m, s, a)
        #@info locs
        loc_prob = gather_info_prob/length(locs)
        for loc in locs
            dist = norm(loc-s.robot)
            # TO-DO model float battery loss as a function of distance (remove round())
            battery_loss = round(dist) == 0.0 ? (m.resolution/25) + gather_info_time : round(dist)*(m.resolution/25) + gather_info_time
            #@info "battery_loss: ", battery_loss
            push!(states, FullState(loc, s.target, copy(s.visited), s.battery-battery_loss))
            push!(probs, loc_prob)
        end
    else
        push!(probs, 1.0)
    end

    for sp ∈ states
        sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
        sp.visited[sind] = 0
    end

    #@info "batt losses: ", [s.battery-sp.battery for sp in states]
    return SparseCat(states, probs)

end

function POMDPs.reward(m::TargetSearchPOMDP{S,A,O}, s::FullState, a::Symbol, sp::FullState) where {S,A,O}
    reward_running = -1.0
    reward_target = 0.0
    reward_nogo = 0.0

    if isequal(sp.robot, sp.target)# if target is found
        reward_running = 0.0
        reward_target = 1000.0 
        return reward_running + reward_target
    end
    if sp.robot ∈ m.obstacles
        reward_nogo = typemin(Float64)
    end
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end

    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        return reward_running + reward_target + m.reward[inds...]*s.visited[spind] + reward_nogo
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


"""
    observations(m::TargetSearchPOMDP)

Retrieve observations in TargetSearch observation space

The the observations are ordered as follows:
    1: The next waypoint is reached
    2: FALCO gather_action is executed
    3: Target is found
"""


POMDPs.observations(m::TargetSearchPOMDP) = OBSERVATIONS #vec(collect(BitVector([c[1],c[2],c[3],c[4],c[5]]) for c in Iterators.product(0:1, 0:1, 0:1, 0:1, 0:1)))
POMDPs.obsindex(m::TargetSearchPOMDP, o::Symbol) = obsind[o]

function POMDPs.observation(m::TargetSearchPOMDP, s::TSState, a::Symbol, sp::TSState)
    
    # TODO: parametrize battery loss for gather info
    if s.battery - sp.battery > m.resolution/25
        return SparseCat(OBSERVATIONS, [0.0, 0.9, 0.1])
    elseif s.battery - sp.battery == m.resolution/25
        return Deterministic(:next_waypoint)
    elseif sp.robot == [-1,-1]
        return Deterministic(:target_found)
    else
        @show s.robot, sp.robot, a
        println(s==sp ? "same state" : "diff state")
        error("Invalid battery loss: ", s.battery - sp.battery)
    end
    
    #=
    
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

    return SparseCat(OBSERVATIONS, probs)
    =#

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

function rewardinds(m, pos::SVector{2, Int64})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]

    return inds
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