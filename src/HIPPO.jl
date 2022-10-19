module HIPPO

using POMDPs
using POMDPTools
using Distributions
import POMDPTools:Uniform as Uni
using Random
using StaticArrays
using Parameters
using LinearAlgebra
using Compose

export 
    TargetSearchPOMDP,
    TSState

struct TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
end

struct roi
    maploc::SVector{2, Int}
    prob::Float64
end

struct TargetSearchPOMDP <: POMDP{TSState, Symbol, BitArray{1}}
    size::SVector{2, Int}
    obstacles::Set{SVector{}}
    robot_init::SVector{2, Int}
    tprob::Float64
    targetloc::SVector{2, Int}
    #rois::Set{}
    #obsindices::Array{Union{Nothing,Int}, 4}
end

function TargetSearchPOMDP(;roipoints=[], size=(10,10), n_obstacles=8, rng::AbstractRNG=Random.MersenneTwister(20))
    obstacles = Set{SVector{2, Int}}()
    robot_init = SVector(rand(rng, 1:size[1]), rand(rng, 1:size[2]))
    tprob = 0.7
    targetloc = SVector(size)
    #= for state in roisstates
        push!(rois, TSState(robot_init, state))
    end
    rois = TSState[] =#

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
    if s.robot == s.target
        return Deterministic(TSState([-1,-1], [-1,-1]))
    end

    states = [TSState(s.robot, m.targetloc)]
    probs = Float64[0.0]
    
    #= if s.robot in m.rois
        push!(states, TSState(SA[-1,-1], SA[-1,-1])) # terminal state for regions of interest
        push!(probs, s.rois[s.robot].prob)
    end =#

    for change in actionvals
        newrobot = bounce(m, s.robot, change)

        if change == actiondir[a]
            if newrobot == s.robot # robot bounced off wall 
                probs[1] += m.tprob
            else 
                push!(probs, m.tprob)
                push!(states, TSState(newrobot, m.targetloc))
            end
        else
            tprob = (1-m.tprob)/(length(actions(m))-1)
            if newrobot == s.robot # robot bounced off wall 
                probs[1] += tprob
            else 
                push!(probs, tprob)
                push!(states, TSState(newrobot, m.targetloc))
            end
        end
    end
    
    return SparseCat(states, probs)

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

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, step)
    nx, ny = m.size
    cells = []
    target_marginal = zeros(nx, ny)
    
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        target = compose(context(), rectangle(), stroke("gray"))
        compose!(cell, target)
        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), fill("white"), stroke("gray"))

    if haskey(step, :sp)
        robot_ctx = cell_ctx(step[:sp].robot, m.size)
        robot = compose(robot_ctx, circle(0.5, 0.5, 0.5), fill("green"))
        target_ctx = cell_ctx(step[:sp].target, m.size)
        target = compose(target_ctx, circle(0.5, 0.5, 0.5), fill("orange"))
    else
        robot = nothing
        target = nothing
    end
    sz = min(w,h)
    return compose(context((w-sz)/2, (h-sz)/2, sz, sz), robot, target, grid, outline)
end

function cell_ctx(xy, size)
    nx, ny = size
    x, y = xy
    return context((x-1)/nx, (ny-y)/ny, 1/nx, 1/ny)
end
POMDPs.isterminal(m::TargetSearchPOMDP, s::TSState) = s.target == s.robot

end # module
