module targetSearch
using Random

export 
    LaserTagPOMDP,
    TSState

struct TSState
    robot::Svector{2, Int}
    target::Svector{2, Int}
end

struct TargetSearchPOMDP <: POMDP{TSState, Symbol, SVector{4,Int}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    #obsindices::Array{Union{Nothing,Int}, 4}
end

function TargetSearchPOMDP(;size=(10,10), n_obstacles=8, rng::AbstractRNG=Random.MersenneTwister(20))
    obstacles = Set{SVector{2, Int}}()
    robot_init = SVector(rand(rng, 1:size[1]), rand(rng, 1:size[2]))

    TargetSearchPOMDP(size, obstacles, robot_init)
end


end