module HIPPO

using POMDPs
using POMDPTools
using Distributions
import POMDPTools:Uniform as Uni
using Random
using StaticArrays
using Parameters
using LinearAlgebra

export 
    TargetSearchPOMDP,
    TSState

struct TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
end

struct TargetSearchPOMDP <: POMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    #obsindices::Array{Union{Nothing,Int}, 4}
end

function TargetSearchPOMDP(;size=(10,10), n_obstacles=8, rng::AbstractRNG=Random.MersenneTwister(20))
    obstacles = Set{SVector{2, Int}}()
    robot_init = SVector(rand(rng, 1:size[1]), rand(rng, 1:size[2]))
    tprob = 0.7
    targetloc = SVector(size)

    TargetSearchPOMDP(size, obstacles, robot_init, tprob, targetloc)
end

POMDPs.states(m::TargetSearchPOMDP) = vec(collect(TSState(SVector(c[1],c[2]), SVector(c[3],c[4])) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2])))
POMDPs.actions(m::TargetSearchPOMDP) = (:left, :right, :up, :down)
include("observations.jl")
POMDPs.discount(m::TargetSearchPOMDP) = 0.95
POMDPs.stateindex(m::TargetSearchPOMDP, s) = LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]


const actiondir = Dict(:left=>SVector(-1,0), :right=>SVector(1,0), :up=>SVector(0, 1), :down=>SVector(0,-1))
const actionind = Dict(:left=>1, :right=>2, :up=>3, :down=>4)
const actionvals = values(actiondir)
const target = SVector()

function bounce(m::TargetSearchPOMDP, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

function POMDPs.transition(m::TargetSearchPOMDP, s, a)
    robotlocs = [TSState(s.robot, m.targetloc)]
    robotprobs = Float64[0.0]
    
    for change in actionvals
        newrobot = bounce(m, s.robot, change)

        if change == actiondir[a]
            if newrobot == s.robot # robot bounced off wall 
                robotprobs[1] += m.tprob
            else 
                push!(robotprobs, m.tprob)
                push!(robotlocs, TSState(newrobot, m.targetloc))
            end
        else
            tprob = (1-m.tprob)/(length(actions(m))-1)
            if newrobot == s.robot # robot bounced off wall 
                robotprobs[1] += tprob
            else 
                push!(robotprobs, tprob)
                push!(robotlocs, TSState(newrobot, m.targetloc))
            end
        end
    end
    
    return SparseCat(robotlocs, robotprobs)

end

function POMDPs.reward(m::TargetSearchPOMDP, s::TSState, a::Symbol, sp::TSState)
    if sp.robot == sp.target # if target is found
        return 100.0 
    end

    return -1.0 # running cost
end

function POMDPs.initialstate(m::TargetSearchPOMDP)
    return POMDPTools.Uniform(TSState(m.robot_init, SVector(x, y)) for x in 1:m.size[1], y in 1:m.size[2])
end

#= function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, step)
    nx, ny = m.size
    cells = []
    target_marginal = zeros(nx, ny)
    if haskey(step, :bp)

end =#

POMDPs.isterminal(m::TargetSearchPOMDP, s::TSState) = s.target == s.robot

end # module
