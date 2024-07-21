"""
    states (some of these functions are not used since the state space is too large)
"""
function POMDPs.stateindex(m::TargetSearchPOMDP, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

function POMDPs.states(m::TargetSearchPOMDP)
    nonterm = vec(collect(UnifiedState(SVector(c[1],c[2]), SVector(c[3],c[4]), BitVector(d), e) 
                    for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]) 
                    for d in Iterators.product(ntuple(s->[0,1],prod(m.size))...))
                    for e in 0:m.maxbatt)

    return push!(nonterm, UnifiedState([-1,-1],[-1,-1],trues(prod(m.size)),-1))
end

function POMDPs.initialstate(m::TargetSearchPOMDP)
    return POMDPTools.Uniform(UnifiedState(m.robot_init, SVector(x, y), trues(prod(m.size)), m.maxbatt, false, m.initial_orientation) for x in 1:m.size[1], y in 1:m.size[2])
end

function POMDPs.initialstate(m::RewardPOMDP)
    return POMDPTools.Uniform(RewardState(m.robot_init, SVector(x, y), trues(prod(m.size))) for x in 1:m.size[1], y in 1:m.size[2])
end

POMDPs.discount(m::TargetSearchPOMDP) = 0.95 

"""
    actions
"""

const PRIMITIVE_ACTIONS = (:left, :right, :up, :down, :stay, :nw, :ne, :sw, :se)

const ORIENTATIONS = [:left, :right, :up, :down, :nw, :ne, :sw, :se]

function POMDPs.actions(m::UnifiedPOMDP, b::LeafNodeBelief)
    o = currentobs(b)
    if o == :low
        return PRIMITIVE_ACTIONS
    elseif o == :medium 
        return PRIMITIVE_ACTIONS
    elseif o == :high
        return (PRIMITIVE_ACTIONS..., unique(Vector.(rand(non_cardinal_states_in_fov(m, b.sp), m.num_macro_actions(b))))...)
    end
    return PRIMITIVE_ACTIONS

end

function POMDPs.actions(m::UnifiedPOMDP, b::ParticleCollection)
    # parametrize macro actions initial state and belief
    current_state = b.particles[1]
    fov_states = non_cardinal_states_in_fov(m, current_state)
    newdict = filter(s -> s[1].target ∈ fov_states, b._probs)
    isempty(newdict) && @warn "---TARGET NOT IN FOV---" && return PRIMITIVE_ACTIONS
    newdist = SparseCat(keys(newdict), values(newdict))
    states = [Vector(s.target) for s ∈ unique(rand(newdist, m.num_macro_actions(b)))]
    isempty(states) && @warn "---NO AVAILABLE MACRO-ACTIONS---" && return PRIMITIVE_ACTIONS
    #@show "BELIEF MACRO-ACTIONS: ", states
    return (PRIMITIVE_ACTIONS..., states...)
end

function states_in_fov(m::UnifiedPOMDP, s::UnifiedState)
    # TODO: create fov as a function of orientation, altitude, and camera angle
    states = []
    # diff = orientdir(s.orientation)
    for i in -2:2
        for j in -2:2
            push!(states, bounce(m, s.robot, SVector(i, j)))
        end
    end
    return unique(states)
end

function cells_in_fov(size, cell, orientation)
    # TODO: create fov as a function of orientation, altitude, and camera angle
    states = []
    # diff = orientdir(s.orientation)
    for i in -2:2
        for j in -2:2
            push!(states, HIPPO.bounce(size, cell, SVector(i, j)))
        end
    end
    return unique(states)
end

function precompute_fov(size, orientations)
    fov_lookup = Dict{Tuple{Int, Int, Symbol}, Vector{Vector{Int}}}()
    for x in 1:size[1], y in 1:size[2], orientation in orientations
        fov = cells_in_fov(size, [x, y], orientation)
        fov_lookup[(x, y, orientation)] = fov
    end
    return fov_lookup
end

function is_in_fov(m, target, robot_state, robot_orientation)
    return target ∈ m.fov_lookup[(robot_state[1], robot_state[2], robot_orientation)]
end

function non_cardinal_states_in_fov(m::UnifiedPOMDP, s::UnifiedState)
    # TODO: create fov as a function of orientation, altitude, and camera angle
    states = []
    # diff = orientdir(s.orientation)
    for i in -2:2
        for j in -2:2
            push!(states, bounce(m, s.robot, SVector(i, j)))
        end
    end
    return filter(x->norm(x-s.robot)>=2.0, unique(states))
end

POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]

POMDPs.actiontype(::Type{<:TargetSearchPOMDP}) = Union{Symbol, Vector{Int}}

const MacroAction = Vector{Int}

function bounce(m::TargetSearchPOMDP, pos, offset)
    if clamp.(pos + offset, SVector(1,1), m.size) ∈ m.obstacles
        return pos
    end

    clamp.(pos + offset, SVector(1,1), m.size)
end

function bounce(size, pos, offset)
    clamp.(pos + offset, SVector(1,1), size)
end

function projected_states(m, s, diff)
    states = []
    for i in 0:8
        push!(states, bounce(m, s.robot, diff + SVector(i%3-1, i÷3-1)))
    end
    return unique(states)
end

# function projected_states(m, s, diff, camera_angle=π/4)
#     states = []
#     ground_distance = 10 * tan(camera_angle)  # Distance on ground from robot to center of view
    
#     # Define the projection plane
#     forward_vector = normalize(SVector(cos(s.robot[4]), sin(s.robot[4]), 0))
#     right_vector = SVector(-forward_vector[2], forward_vector[1], 0)
    
#     for i in -1:1, j in -1:1
#         # Calculate offset in the projection plane
#         offset = ground_distance * forward_vector + i * diff[1] * right_vector + j * diff[2] * forward_vector
        
#         # Project this point to the ground
#         ground_point = s.robot[1:2] + offset[1:2]
        
#         # Use the bounce function to get the final state
#         projected_state = bounce(m, SVector(ground_point[1], ground_point[2], s.robot[3], s.robot[4]), SVector(0, 0))
        
#         push!(states, projected_state)
#     end
    
#     return unique(states)
# end
function projected_state_random(m, s)
    states = []
    for i in 0:8
        push!(states, bounce(m, s.robot, SVector(i%3-1, i÷3-1)))
    end
    return rand(unique(states))
end

function POMDPs.transition(m::TargetSearchPOMDP, s::UnifiedState, a::Symbol)
    states = UnifiedState[]
    probs = Float64[]
    nominal_battery_loss = (m.resolution/25)

    sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
    s.visited[sind] = 0

    if isequal(s.robot, s.target)
        return Deterministic(UnifiedState(SA[-1,-1], copy(s.target), s.visited, s.battery, true, s.orientation))
    end
    
    newrobot = bounce(m, s.robot, actiondir[a])

    # TO-DO model float battery loss as a function of distance (remove round())
    push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-nominal_battery_loss, false, a))
    push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-nominal_battery_loss, true, a))
    push!(probs, 0.8, 0.2)
    
    return SparseCat(states, probs)

end

function POMDPs.transition(m::TargetSearchPOMDP, s::UnifiedState, a::MacroAction)
    states = UnifiedState[]
    probs = Float64[]
    gather_info_time = 2*(m.resolution/25)

    sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
    s.visited[sind] = 0

    if isequal(s.robot, s.target)
        return Deterministic(UnifiedState(SA[-1,-1], copy(s.target), s.visited, s.battery, true, s.orientation))
    end
    
    newrobot = a
    traversed_cells = cell_list(s.robot, a)
    for cell in traversed_cells[1:end-1]
        s.visited[LinearIndices((1:m.size[1], 1:m.size[2]))[cell...]] = 0
    end
    
    new_orientation = orientdir[traversed_cells[end] - traversed_cells[end-1]]

    if new_orientation == :stay
        new_orientation = s.orientation
    end

    distance = norm(newrobot-s.robot)

    # TODO: model float battery loss as a function of distance (remove round())

    # TODO: human_in_fov should be known from target and orientation
    human_in_fov = is_in_fov(m, s.target, s.robot, s.orientation)

    battery_loss = round(distance) == 0.0 ? (m.resolution/25) + gather_info_time : round(distance)*(m.resolution/25) + gather_info_time
    push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-battery_loss, human_in_fov, new_orientation))
    # push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-battery_loss, true, new_orientation))
    # push!(probs, 0.2, 0.8)
    push!(probs, 1.0)
   
    return SparseCat(states, probs)

end

"""
    observations(m::TargetSearchPOMDP)

Retrieve observations in TargetSearch observation space

The the observations are ordered as follows:
    1: low detector confidence score: score <= 0.33 (this includes no detection)
    2: medium detector confidence score: 0.33 < score <= 0.66
    3: high detector confidence score: score > 0.66
"""


POMDPs.observations(m::TargetSearchPOMDP) = E2E_OBSERVATIONS 
POMDPs.obsindex(m::TargetSearchPOMDP, o::Symbol) = obsind[o]

function proximal(sp, a)
    if norm(sp.robot-sp.target) <= 2.0
        return true
    else
        return false
    end
    # if sp.robot[1]-sp.target[1] >= 2.0 && a == :left 
    #     return true
    # elseif sp.robot[1]-sp.target[1] <= 2.0 && a == :right
    #     return true
    # elseif sp.robot[2]-sp.target[2] >= 2.0 && a == :down
    #     return true
    # elseif sp.robot[2]-sp.target[2] <= 2.0 && a == :up
    #     return true
    # elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :sw
    #     return true
    # elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :se
    #     return true
    # elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :nw
    #     return true
    # elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :ne
    #     return true
    # else
    #     return false
    # end
end

#POMDPs.observation(m::TargetSearchPOMDP, s::UnifiedState, a::Symbol, sp::UnifiedState) = observation(m, a, sp)

function POMDPs.observation(m::TargetSearchPOMDP, a::Union{Symbol,Vector{Int}}, sp::UnifiedState)
    if proximal(sp, a) 
        if isa(a, MacroAction)
            if sp.human_in_fov # not function of action taken
                return SparseCat(E2E_OBSERVATIONS, [0.01, 0.09, 0.9])
            else    
                return SparseCat(E2E_OBSERVATIONS, [0.9, 0.09, 0.01])
            end
        else
            if sp.human_in_fov # not function of action taken
                return SparseCat(E2E_OBSERVATIONS, [0.1, 0.2, 0.7])
            else    
                return SparseCat(E2E_OBSERVATIONS, [0.95, 0.04, 0.01])
            end
        end
    else
        return SparseCat(E2E_OBSERVATIONS, [0.9, 0.09, 0.01])
    end

    # if norm(sp.robot-sp.target) <= 2.0
    #     if sp.robot[1]-sp.target[1] >= 2.0 && a == :left 
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif sp.robot[1]-sp.target[1] <= 2.0 && a == :right
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif sp.robot[2]-sp.target[2] >= 2.0 && a == :down
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif sp.robot[2]-sp.target[2] <= 2.0 && a == :up
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :sw
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] >= 2.0) && a == :se
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif (sp.robot[1]-sp.target[1] >= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :nw
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     elseif (sp.robot[1]-sp.target[1] <= 2.0) && (sp.robot[2]-sp.target[2] <= 2.0) && a == :ne
    #         return SparseCat(OBSERVATIONS, [0.2, 0.8])
    #     else

    #         return SparseCat(OBSERVATIONS, [0.95, 0.05])
    #     end
    # else
    #     return SparseCat(OBSERVATIONS, [0.95, 0.05])
    # end

    # return SparseCat(OBSERVATIONS, probs)
   

end

function POMDPs.reward(m::TargetSearchPOMDP, s::UnifiedState, a::Symbol, sp::UnifiedState)
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end
    
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
    
    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        return reward_running + reward_target + m.reward[inds...]*sp.visited[spind] + reward_nogo
    end
end

function POMDPs.reward(m::TargetSearchPOMDP, s::UnifiedState, a::MacroAction, sp::UnifiedState)
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end

    rtot = 0.0
    cells = cell_list(s.robot, a)

    reward_running = -1.0
    reward_target = 0.0
    reward_nogo = 0.0

    if sp.robot ∈ m.obstacles
        reward_nogo = typemin(Float64)
    end
    
    for cell in cells
        inds = rewardinds(m, cell)
        spind = LinearIndices((1:m.size[1], 1:m.size[2]))[cell...]
        reward_target = isequal(cell, sp.target) ? 1000.0 : 0.0
        rtot += reward_running + reward_target + m.reward[inds...]*sp.visited[spind] + reward_nogo # THIS DOESN'T CHECK FOR TERMINAL STATE
    end
    return rtot * discount(m)^length(cells)
end

function rewardinds(m, pos::Union{SVector{2, Int64},Vector{Int}})
    correct_ind = reverse(pos)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]

    return inds
end

function dist(curr, start)
    sum(abs.(curr-start))
end

function POMDPs.isterminal(m::TargetSearchPOMDP, s::UnifiedState)
    required_batt = dist(s.robot, m.robot_init)
    return s.battery - required_batt <= 1 || s.robot == s.target#SA[-1,-1] 
end

POMDPs.isterminal(m::RewardPOMDP, s::UnifiedState) = s.robot == SA[-1,-1]

POMDPs.isterminal(m::TargetSearchPOMDP, s::UnifiedState, target) = s.robot == target