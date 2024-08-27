"""
    states (some of these functions are not used since the state space is too large)
"""
function POMDPs.stateindex(m::UnifiedPOMDP, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

function POMDPs.states(m::UnifiedPOMDP)
    nonterm = vec(collect(UnifiedState(SVector(c[1],c[2]), SVector(c[3],c[4]), BitVector(d), e) 
                    for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]) 
                    for d in Iterators.product(ntuple(s->[0,1],prod(m.size))...))
                    for e in 0:m.maxbatt)

    return push!(nonterm, UnifiedState([-1,-1],[-1,-1],trues(prod(m.size)),-1))
end

function POMDPs.initialstate(m::UnifiedPOMDP)
    return POMDPTools.Uniform(UnifiedState(m.robot_init, SVector(x, y), trues(prod(m.size)), m.maxbatt, false, m.initial_orientation) for x in 1:m.size[1], y in 1:m.size[2])
end

POMDPs.discount(m::UnifiedPOMDP) = 1.0

discount_factor(m::UnifiedPOMDP) = 0.95

"""
    actions
"""

const PRIMITIVE_ACTIONS = (:left, :right, :up, :down, :nw, :ne, :sw, :se)

const FULL_ACTIONS = (:left, :right, :up, :down, :nw, :ne, :sw, :se, :investigate)

function POMDPs.actions(m::UnifiedPOMDP)
    return PRIMITIVE_ACTIONS
end

# function POMDPs.actions(m::UnifiedPOMDP, b::LeafNodeBelief)
#     o = currentobs(b)
#     if o == :low
#         return PRIMITIVE_ACTIONS
#     elseif o == :medium 
#         return PRIMITIVE_ACTIONS
#     elseif o == :high
#         fov_cells = non_cardinal_states_in_fov(m, b.sp)
#         if !isterminal(m, b.sp) && !isempty(fov_cells)
#             return (PRIMITIVE_ACTIONS..., unique(Vector.(rand(non_cardinal_states_in_fov(m, b.sp), m.num_macro_actions(b))))...)
#         end
#     end
#     return PRIMITIVE_ACTIONS
# end
function POMDPs.actions(m::UnifiedPOMDP, b::LeafNodeBelief)
    o = currentobs(b)
    if o == :low
        return PRIMITIVE_ACTIONS
    elseif o == :medium 
        return PRIMITIVE_ACTIONS
    elseif o == :high
        #if b.sp.human_in_fov && rand() < 0.3
        #    return [Vector(b.sp.target)]
        #end
        fov_cells = non_cardinal_states_in_fov(m, b.sp)
        if !isterminal(m, b.sp) && !isempty(fov_cells)
            return (PRIMITIVE_ACTIONS..., unique(Vector.(rand(non_cardinal_states_in_fov(m, b.sp), m.num_macro_actions(b))))...)
        end
    end
    return PRIMITIVE_ACTIONS
end


function POMDPs.actions(m::UnifiedPOMDP, b::ParticleCollection)
    # parametrize macro actions initial state and belief
    robot_state = b.particles[1]
    fov_states = non_cardinal_states_in_fov(m, robot_state)
    #@info robot_state.robot, fov_states
    newdict = filter(s -> s[1].target ∈ fov_states, ParticleFilters.probdict(b))
    if isempty(newdict)
        #@warn "---TARGET NOT IN FOV---"
        return PRIMITIVE_ACTIONS
    end
    newdist = SparseCat(keys(newdict), values(newdict))
    states = [Vector(s.target) for s ∈ unique(rand(newdist, m.num_macro_actions(b)))]
    isempty(states) && @warn "---NO AVAILABLE MACRO-ACTIONS---" && return PRIMITIVE_ACTIONS
    #@show "BELIEF MACRO-ACTIONS: ", states
    return (PRIMITIVE_ACTIONS..., states...)
        
end

function states_in_fov(m::UnifiedPOMDP, s::UnifiedState, pose::RobotPose)
    pose.x = s.robot[1] * m.resolution
    pose.y = s.robot[2] * m.resolution
    return states_in_fov(m, pose)
end

function states_in_fov(m::UnifiedPOMDP, pose::RobotPose)
    bbox = getBoundingPolygon(m.camera_info, pose)
    return project_footprint_to_grid(bbox, m.size[1], m.size[2], m.resolution)
end

function non_cardinal_states_in_fov(m::UnifiedPOMDP, s::UnifiedState)
    m.pose.x = (s.robot[1] - 0.5) * m.resolution
    m.pose.y = (s.robot[2] - 0.5) * m.resolution
    m.pose.heading = headingdir[s.orientation]
    bbox = getBoundingPolygon(m.camera_info, m.pose)
    states = project_footprint_to_grid(bbox, m.size[1], m.size[2], m.resolution)
    return filter(x->norm(x-s.robot)>=2.0, states)
end
function non_cardinal_states_in_fov(camera_info, pose, size, resolution)
    bbox = getBoundingPolygon(camera_info, pose)
    states = project_footprint_to_grid(bbox, size[1], size[2], resolution)
    return filter(robotpos->norm(robotpos-[pose.x, pose.y])>=2.0, states)
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

function precompute_camera_footprint(cam_info, pose, resolution, size, orientations)
    fov_lookup = Dict{Tuple{Int, Int, Symbol}, Vector{Vector{Int}}}()
    for x in 1:size[1], y in 1:size[2], orientation in orientations
        pose.x = (x - 0.5) * resolution
        pose.y = (y - 0.5) * resolution
        pose.heading = headingdir[orientation]
        fov = non_cardinal_states_in_fov(cam_info, pose, size, resolution)
        fov_lookup[(x, y, orientation)] = fov
    end
    return fov_lookup
end

function is_in_fov(m, target, robot_state, robot_orientation)
    return target ∈ m.fov_lookup[(robot_state[1], robot_state[2], robot_orientation)]
end

# function non_cardinal_states_in_fov(m::UnifiedPOMDP, s::UnifiedState)
#     # TODO: create fov as a function of orientation, altitude, and camera angle
#     states = []
#     # diff = orientdir(s.orientation)
#     for i in -2:2
#         for j in -2:2
#             push!(states, bounce(m, s.robot, SVector(i, j)))
#         end
#     end
#     return filter(x->norm(x-s.robot)>=2.0, unique(states))
# end

POMDPs.actionindex(m::UnifiedPOMDP, a) = actionind[a]

POMDPs.actiontype(::Type{<:UnifiedPOMDP}) = Union{Symbol, Vector{Int}}

const MacroAction = Vector{Int}

function bounce(m::UnifiedPOMDP, pos, offset)
    if clamp.(pos + offset, SVector(1,1), m.size) ∈ m.obstacles
        return pos
    end

    clamp.(pos + offset, SVector(1,1), m.size)
end

function bounce(size, pos, offset)
    clamp.(pos + offset, SVector(1,1), size)
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

function POMDPs.transition(m::UnifiedPOMDP, s::UnifiedState, a::Symbol)
    states = UnifiedState[]
    probs = Float64[]
    nominal_battery_loss = (m.resolution/25)

    sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
    s.visited[sind] = 0

    if isequal(s.robot, s.target)
        return Deterministic(UnifiedState(SA[-1,-1], copy(s.target), s.visited, s.battery, true, s.orientation))
    end
    
    newrobot = bounce(m, s.robot, actiondir[a])

    human_in_fov = is_in_fov(m, s.target, newrobot, a)

    # TO-DO model float battery loss as a function of distance (remove round())
    push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-nominal_battery_loss, human_in_fov, a))
    push!(probs, 1.0)
    
    return SparseCat(states, probs)

end

function POMDPs.transition(m::UnifiedPOMDP, s::UnifiedState, a::MacroAction)
    states = UnifiedState[]
    probs = Float64[]
    gather_info_time = 2*(m.resolution/25)
    nominal_battery_loss = (m.resolution/25)

    sind = LinearIndices((1:m.size[1], 1:m.size[2]))[s.robot...]
    s.visited[sind] = 0

    if isequal(s.robot, s.target)
        return Deterministic(UnifiedState(SA[-1,-1], copy(s.target), s.visited, s.battery, true, s.orientation))
    end
    
    newrobot = a
    traversed_cells = cell_list(s.robot, a)
    for i ∈ eachindex(traversed_cells[1:end-1])
        if isequal(traversed_cells[i], s.target) || s.battery - (i*nominal_battery_loss) == 0.0
            return Deterministic(UnifiedState(SA[-1,-1], copy(s.target), s.visited, s.battery - (i * nominal_battery_loss), true, s.orientation))
        end
        s.visited[LinearIndices((1:m.size[1], 1:m.size[2]))[traversed_cells[i]...]] = 0
    end
    
    try
        new_orientation = orientdir[traversed_cells[end] - traversed_cells[end-1]]
    catch
        @info s 
        println("ERROR: ", s.robot, " | ", a, " | ", traversed_cells)
    end
    if new_orientation == :stay
        new_orientation = s.orientation
    end

    distance = norm(newrobot-s.robot)

    # TODO: model float battery loss as a function of distance (remove round())

    # TODO: human_in_fov should be known from target and orientation
    human_in_fov = is_in_fov(m, s.target, s.robot, new_orientation)
    
    battery_loss = round(distance) == 0.0 ? (m.resolution/25) + gather_info_time : round(distance)*(m.resolution/25) + gather_info_time
    push!(states, UnifiedState(newrobot, s.target, copy(s.visited), s.battery-battery_loss, human_in_fov, new_orientation))
   
    # if is_in_fov(m, s.target, s.robot, s.orientation)
    #     push!(probs, 0.7)
    #     distance = norm(s.target-s.robot)
    #     battery_loss = round(distance) == 0.0 ? (m.resolution/25) + gather_info_time : round(distance)*(m.resolution/25) + gather_info_time
    #     push!(states, UnifiedState(s.target, s.target, copy(s.visited), s.battery - battery_loss, false, new_orientation))
    #     push!(probs, 0.3)
    # else
    #     push!(probs, 1.0)
    # end
    push!(probs, 1.0)
   
    return SparseCat(states, probs)
end

"""
    observations(m::UnifiedPOMDP)

Retrieve observations in TargetSearch observation space

The the observations are ordered as follows:
    1: low detector confidence score: score <= 0.33 (this includes no detection)
    2: medium detector confidence score: 0.33 < score <= 0.66
    3: high detector confidence score: score > 0.66
"""


POMDPs.observations(m::UnifiedPOMDP) = E2E_OBSERVATIONS 
POMDPs.obsindex(m::UnifiedPOMDP, o::Symbol) = obsind[o]

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

#POMDPs.observation(m::UnifiedPOMDP, s::UnifiedState, a::Symbol, sp::UnifiedState) = observation(m, a, sp)

function POMDPs.observation(m::UnifiedPOMDP, a::Union{Symbol,Vector{Int}}, sp::UnifiedState)
    # if proximal(sp, a) 
    #     if isa(a, MacroAction)
    #         if sp.human_in_fov # not function of action taken
    #             return SparseCat(E2E_OBSERVATIONS, [0.01, 0.09, 0.9])
    #         else    
    #             return SparseCat(E2E_OBSERVATIONS, [0.9, 0.09, 0.01])
    #         end
    #     else
    #         if sp.human_in_fov # not function of action taken
    #             return SparseCat(E2E_OBSERVATIONS, [0.1, 0.2, 0.7])
    #         else    
    #             return SparseCat(E2E_OBSERVATIONS, [0.95, 0.04, 0.01])
    #         end
    #     end
    # else
    #     return SparseCat(E2E_OBSERVATIONS, [0.9, 0.09, 0.01])
    # end

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

function POMDPs.reward(m::UnifiedPOMDP, s::UnifiedState, a::Symbol, sp::UnifiedState)
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end
    
    γ_horizon = discount_factor(m)^(m.currentbatt-sp.battery) 

    reward_running = -1.0
    reward_target = 0.0
    reward_nogo = 0.0

    if isequal(sp.robot, sp.target)# if target is found
        reward_running = 0.0
        reward_target = 1000.0 
        #return reward_running + reward_target
    end

    if sp.robot ∈ m.obstacles
        reward_nogo = typemin(Float64)
    end
    
    inds = rewardinds(m, sp.robot)
    spind = LinearIndices((1:m.size[1], 1:m.size[2]))[sp.robot...]

    if !isempty(m.reward) && sp.robot != SA[-1,-1]
        return γ_horizon * (reward_running + reward_target + m.reward[inds...]*sp.visited[spind] + reward_nogo)
    end
end

function POMDPs.reward(m::UnifiedPOMDP, s::UnifiedState, a::MacroAction, sp::UnifiedState)
    if isterminal(m, sp) # IS THIS NECCESSARY?
        return 0.0
    end
    
    rtot = 0.0
    cells = cell_list(s.robot, a)

    reward_running = -1.0
    reward_target = 0.0
    reward_nogo = 0.0
    #reward_investigate = 0.0

    if sp.robot ∈ m.obstacles
        reward_nogo = typemin(Float64)
    end

    if sp.human_in_fov
        reward_investigate = 1000.0
    else
        reward_investigate = -1000.0
    end

    # TODO: CHECK FOR TERMINAL STATE
    if s.robot == [-1,-1]
        for i ∈ 1:s.battery-sp.battery
            if isequal(cells[i], sp.target)
                reward_target = 1000.0
            end
            rtot += discount_factor(m)^(m.currentbatt-(s.battery-i)) * (reward_running + reward_target + reward_nogo)
        end
        return rtot += discount_factor(m)^(m.currentbatt-s.battery) * reward_investigate
    end

    for i ∈ eachindex(cells)
        reward_target = isequal(cells[i], sp.target) ? 1000.0 : 0.0
        rtot += discount_factor(m)^(m.currentbatt-(s.battery-i)) * (reward_running + reward_target + reward_nogo)
    end

    return rtot += discount_factor(m)^(m.currentbatt-s.battery) * reward_investigate#rtot * discount(m)^length(cells)
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

function POMDPs.isterminal(m::UnifiedPOMDP, s::UnifiedState)
    required_batt = dist(s.robot, m.robot_init)
    return s.battery - required_batt <= 1 || s.robot == SA[-1,-1] # s.target
end

POMDPs.isterminal(m::RewardPOMDP, s::UnifiedState) = s.robot == SA[-1,-1]

#POMDPs.isterminal(m::UnifiedPOMDP, s::UnifiedState, target) = s.robot == target