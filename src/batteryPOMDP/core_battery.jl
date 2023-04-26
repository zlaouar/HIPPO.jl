struct TSStateBattery
    robot::SVector{2, Int}
    target::SVector{2, Int}
    battery::Int
end

mutable struct TSPOMDPBattery <: POMDP{TSStateBattery, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    rois::Dict{Vector{Int64}, Float64}
    belief::Matrix{Float64}
    maxbatt::Int
    #obsindices::Array{Union{Nothing,Int}, 4}
end

function TSPOMDPBattery(sinit::TSStateBattery; 
                        roi_points=Dict(), 
                        size=(10,10), 
                        belief=Array{Float64}(undef, 0, 0), 
                        maxbatt=100)

    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points
    maxbatt = maxbatt
  
    TSPOMDPBattery(size, obstacles, robot_init, tprob, targetloc, rois, belief, maxbatt)
end