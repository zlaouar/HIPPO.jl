struct TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
    visited::BitVector
end

mutable struct TargetSearchPOMDP <: POMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    reward::Matrix{Float64}
    #obsindices::Array{Union{Nothing,Int}, 4}
end

function TargetSearchPOMDP(sinit::TSState; roi_points=Dict(), size=(10,10), rewarddist=Array{Float64}(undef, 0, 0))
    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points
  
    TargetSearchPOMDP(size, obstacles, robot_init, tprob, targetloc, rois, copy(rewarddist))
end