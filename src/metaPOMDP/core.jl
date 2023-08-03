abstract type TSState end

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

abstract type TargetSearchPOMDP <: POMDP{TSState, Symbol, BitArray{1}} end

mutable struct BasicPOMDP <: TargetSearchPOMDP
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
end

mutable struct RewardPOMDP <: TargetSearchPOMDP
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
end

mutable struct BatteryPOMDP <:TargetSearchPOMDP
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    maxbatt::Int
end

mutable struct FullPOMDP <: TargetSearchPOMDP
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
    maxbatt::Int
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
