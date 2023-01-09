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
    TSState,
    roi

struct TSState
    robot::SVector{2, Int}
    target::SVector{2, Int}
end

#= struct roi
    maploc::SVector{2, Int}
    prob::Float64
end
 =#
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

function TargetSearchPOMDP(;roi_points=Dict(), size=(10,10), sinit=TSState([10,1],[2,7]), rng::AbstractRNG=Random.MersenneTwister(20), rewarddist=Array{Float64}(undef, 0, 0))
    obstacles = Set{SVector{2, Int}}()
    robot_init = sinit.robot
    tprob = 0.7
    targetloc = sinit.target
    rois = roi_points
  
    TargetSearchPOMDP(size, obstacles, robot_init, tprob, targetloc, rois, rewarddist)
end

function POMDPs.states(m::TargetSearchPOMDP) 
    nonterm = vec(collect(TSState(SVector(c[1],c[2]), SVector(c[3],c[4])) for c in Iterators.product(1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2])))
    return push!(nonterm, TSState([-1,-1],[-1,-1]))
end

POMDPs.actions(m::TargetSearchPOMDP) = (:left, :right, :up, :down, :stay)
include("observations.jl")
POMDPs.discount(m::TargetSearchPOMDP) = 0.95

function POMDPs.stateindex(m::TargetSearchPOMDP, s)
    if s.robot == SA[-1,-1]
        return m.size[1]^2 * m.size[2]^2 + 1
    else 
        return LinearIndices((1:m.size[1], 1:m.size[2], 1:m.size[1], 1:m.size[2]))[s.robot..., s.target...]
    end
end

POMDPs.actionindex(m::TargetSearchPOMDP, a) = actionind[a]


const actiondir = Dict(:left=>SVector(-1,0), :right=>SVector(1,0), :up=>SVector(0, 1), :down=>SVector(0,-1), :stay=>SVector(0,0))
const actionind = Dict(:left=>1, :right=>2, :up=>3, :down=>4, :stay=>5)
const actionvals = values(actiondir)
#const target = SVector()

function bounce(m::TargetSearchPOMDP, pos, offset)
    new = clamp.(pos + offset, SVector(1,1), m.size)
end

#= function POMDPs.transition(m::TargetSearchPOMDP, s, a)
    if s.robot == s.target
        return Deterministic(TSState([-1,-1], [-1,-1]))
    end

    states = [TSState(s.robot, m.targetloc)]
    probs = Float64[0.0]
    tprob = m.tprob
    off_main_prob = m.tprob

    if haskey(m.rois, s.robot)
        push!(states, TSState(SA[-1,-1], SA[-1,-1])) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
        tprob = m.tprob * remaining_prob
        off_main_prob += tprob
    end

    for change in actionvals
        newrobot = bounce(m, s.robot, change)

        if change == actiondir[a]
            if newrobot == s.robot # robot bounced off wall 
                probs[1] += tprob
            else 
                push!(probs, tprob)
                push!(states, TSState(newrobot, m.targetloc))
            end
        else
            tprob = (1-off_main_prob)/(length(actions(m))-1)
            if newrobot == s.robot # robot bounced off wall 
                probs[1] += tprob
            else 
                push!(probs, tprob)
                push!(states, TSState(newrobot, m.targetloc))
            end
        end
    end

    return SparseCat(states, probs)

end =#

function POMDPs.transition(m::TargetSearchPOMDP, s, a)
    #= if s.robot == s.target
        return Deterministic(TSState([-1,-1], [-1,-1]))
    end=#

    states = TSState[]
    probs = Float64[]
    remaining_prob = 1.0

    if haskey(m.rois, s.robot)
        push!(states, TSState(SA[-1,-1], SA[-1,-1])) # terminal state for regions of interest
        push!(probs, m.rois[s.robot])
        #display(m.rois[s.robot])
        remaining_prob = 1-m.rois[s.robot]
    end


    newrobot = bounce(m, s.robot, actiondir[a])

    push!(states, TSState(newrobot, s.target))
    push!(probs, remaining_prob)

    #display(probs)
    #display(SparseCat(states, probs))

    return SparseCat(states, probs)

end

function POMDPs.reward(m::TargetSearchPOMDP, s::TSState, a::Symbol, sp::TSState)
    rmat = m.reward

    correct_ind = reverse(s.robot)
    xind = m.size[2]+1 - correct_ind[1]
    inds = [xind, correct_ind[2]]
    
    reward_running = -1.0
    reward_target = 0.0
    reward_roi = 0.0
    if sp.robot == sp.target && sp.robot != SA[-1,-1]# if target is found
        reward_running = 0.0
        reward_target = 100.0 
    end
    if sp.robot == SA[-1,-1]
        reward_running = 0.0
        reward_roi = 0.0
    end
    #m.reward[inds...] = 0.0
    if !isempty(rmat)
        return reward_running + reward_target + reward_roi + rmat[inds...] # running cost
    else
        reward_running + reward_target + reward_roi
    end
end

function POMDPs.initialstate(m::TargetSearchPOMDP)
    return POMDPTools.Uniform(TSState(m.robot_init, SVector(x, y)) for x in 1:m.size[1], y in 1:m.size[2])
end

function POMDPTools.ModelTools.render(m::TargetSearchPOMDP, step)
    nx, ny = m.size
    cells = []
    target_marginal = zeros(nx, ny)
    rois = collect(keys(m.rois))

    if haskey(step, :bp) && !ismissing(step[:bp])
        for sp in support(step[:bp])
            p = pdf(step[:bp], sp)
            if sp.target != [-1,-1] # TO-DO Fix this
                target_marginal[sp.target...] += p
            end
        end
    end
    for x in 1:nx, y in 1:ny
        cell = cell_ctx((x,y), m.size)
        t_op = sqrt(target_marginal[x,y])
        # TO-DO Fix This
        if t_op > 1.0
            t_op = 0.999
        end
        
        target = compose(context(), rectangle(), fillopacity(t_op), fill("yellow"), stroke("gray"))
        if [x,y] in rois
            roi = compose(context(), rectangle(), fill("transparent"), stroke("white"), linewidth(1.2mm))
            compose!(cell, target, roi)
        else
            compose!(cell, target)
        end

        push!(cells, cell)
    end
    grid = compose(context(), linewidth(0.5mm), cells...)
    outline = compose(context(), linewidth(1mm), rectangle(), fill("black"), stroke("gray"))

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

function POMDPs.isterminal(m::TargetSearchPOMDP, s::TSState)  
    if s.target == s.robot
        return true
    elseif s.robot == SA[-1,-1]
        return true
    else
        return false
    end
end

end

# module
