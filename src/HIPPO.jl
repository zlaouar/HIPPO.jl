module HIPPO

using POMDPs
using POMDPTools
using Distributions
using Random
using StaticArrays
using Parameters


export 
    TargetSearchPOMDP,
    TSState

struct TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
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

POMDPs.states(m::TargetSearchPOMDP) = vec(collect(TSState(SVector(c[1],c[2]), SVector(c[3],c[4])) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2])))
POMDPs.actions(m::TargetSearchPOMDP) = (:left, :right, :up, :down)
include("observations.jl")
POMDPs.discount(m::TargetSearchPOMDP) = 0.95
POMDPs.stateindex(m::TargetSearchPOMDP, s) = LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]
#POMDPs.obsindex(m::TargetSearchPOMDP, o) = m.obsindices

const actiondir = Dict(:left=>SVector(-1,0), :right=>SVector(1,0), :up=>SVector(0, 1), :down=>SVector(0,-1))
const actionind = Dict(:left=>1, :right=>2, :up=>3, :down=>4)

function bounce(m::TargetSearchPOMDP, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

function POMDPs.transition(m::TargetSearchPOMDP, s, a)
    newrobot = bounce(m, s.robot, actiondir[a])


end


end # module
