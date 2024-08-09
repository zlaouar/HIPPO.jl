abstract type TSState end
abstract type PachTemplatePOMDP end
abstract type TargetSearchPOMDP{S,A,O}  <: POMDP{S,A,O} end

struct BasicState <: TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
end

struct RewardState <: TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
    visited::BitVector
end

struct BatteryState <: TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
    battery::Int
end

struct FullState <: TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
    visited::BitVector
    battery::Int
end

struct UnifiedState <: TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
    visited::BitVector
    battery::Int
    human_in_fov::Bool
    orientation::Symbol
end


mutable struct BasicPOMDP <: TargetSearchPOMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
end

mutable struct RewardPOMDP <: TargetSearchPOMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
end

mutable struct BatteryPOMDP <:TargetSearchPOMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    maxbatt::Int
end

mutable struct FullPOMDP <: TargetSearchPOMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
    maxbatt::Int
end

mutable struct PachPOMDP{O} <: TargetSearchPOMDP{TSState, Symbol, O}
    size::SVector{2, Int}
    obstacles::Vector{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
    maxbatt::Int
    resolution::Int
end

mutable struct UnifiedPOMDP{O, F<:Function} <: TargetSearchPOMDP{TSState, Symbol, O}
    size::SVector{2, Int}
    obstacles::Vector{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
    maxbatt::Int
    resolution::Int
    num_macro_actions::F
    initial_orientation::Symbol
    fov_lookup::Dict{Tuple{Int, Int, Symbol}, Vector{Vector{Int}}}
    rollout_depth::Int
    camera_info::CameraInfod
    pose::RobotPose
end

function obs_type(options)
    if options[:observation_model] == :falco
        return Symbol
    else
        return BitArray{1}
    end
end

function create_target_search_pomdp(sinit::TSState; 
                                    roi_points=Dict(), 
                                    size=(10,10), 
                                    rewarddist=Array{Float64}(undef, 0, 0),
                                    maxbatt=100, 
                                    options=Dict(:observation_model=>:falco),
                                    obstacles=Vector{SVector{2, Int}}(),
                                    resolution=25)

    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    PachPOMDP{obs_type(options)}(size, obstacles, robot_init, tprob, 
                                                targetloc, rois, copy(rewarddist), 
                                                maxbatt, resolution)
end

function UnifiedPOMDP(sinit::TSState; 
        roi_points=Dict(), 
        size=(10,10), 
        rewarddist=Array{Float64}(undef, 0, 0),
        maxbatt=100, 
        options=Dict(:observation_model=>:falco),
        obstacles=Vector{SVector{2, Int}}(),
        resolution=25,
        num_macro_actions=(b) -> 4,
        initial_orientation=:up,
        rollout_depth=prod(size),
        camera_info=CameraInfo(deg2rad(71.5), 
                                deg2rad(56.8), 
                                30.0, 
                                deg2rad(0.0), 
                                deg2rad(0.0), 
                                deg2rad(0.0)),
        pose=RobotPose(0.0, 0.0, 30.0, deg2rad(0.0), deg2rad(-45.0), deg2rad(0.0)))

    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    pose.x = (robot_init[1] - 0.5) * resolution
    pose.y = (robot_init[2] - 0.5) * resolution
    pose.heading = headingdir[initial_orientation]

    fov_lookup = precompute_camera_footprint(camera_info, pose, resolution, size, ORIENTATIONS)
    # fov_lookup = precompute_fov(size, ORIENTATIONS)

    UnifiedPOMDP{obs_type(options), typeof(num_macro_actions)}(size, obstacles, robot_init, tprob, 
                                    targetloc, rois, copy(rewarddist), maxbatt, resolution, 
                                    num_macro_actions, initial_orientation, fov_lookup, 
                                    rollout_depth, camera_info, pose)
end

function BasicPOMDP(sinit::BasicState; 
                    roi_points=Dict(), 
                    size=(10,10))

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    BasicPOMDP(size, obstacles, robot_init, tprob, targetloc, rois)
end

function RewardPOMDP(sinit::RewardState; 
                        roi_points=Dict(), 
                        size=(10,10), 
                        rewarddist=Array{Float64}(undef, 0, 0))

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    RewardPOMDP(size, obstacles, robot_init, tprob, targetloc, rois, copy(rewarddist))
end


function BatteryPOMDP(sinit::BatteryState; 
                        roi_points=Dict(), 
                        size=(10,10), 
                        maxbatt=100)

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points
    maxbatt = maxbatt

    BatteryPOMDP(size, obstacles, robot_init, tprob, targetloc, rois, maxbatt)
end

function FullPOMDP(sinit::FullState; 
                    roi_points=Dict(), 
                    size=(10,10), 
                    rewarddist=Array{Float64}(undef, 0, 0),
                    maxbatt=100)

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    FullPOMDP(size, obstacles, robot_init, tprob, targetloc, rois, copy(rewarddist), maxbatt)
end

function PachPOMDP(sinit::FullState; 
                    roi_points=Dict(), 
                    size=(10,10), 
                    rewarddist=Array{Float64}(undef, 0, 0),
                    maxbatt=100,
                    resolution=25)

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points

    PachPOMDP(size, obstacles, robot_init, tprob, targetloc, 
                rois, copy(rewarddist), maxbatt, resolution)
end
