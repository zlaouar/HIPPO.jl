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
using BasicPOMCP
using MCTS
using DiscreteValueIteration

export 
    TSPOMDPBasic,
    TSStateBasic,

    TargetSearchPOMDP,
    TSState,

    BFORollout,
    SolvedBFORollout,
    estimate_value,
    convert_estimator,
    simulate

include(joinpath(@__DIR__,"basic","core_vanilla.jl"))
include(joinpath(@__DIR__,"basic","functions_vanilla.jl"))
include(joinpath(@__DIR__,"basic","observations_vanilla.jl"))

include("core.jl")
include("functions.jl")
include("observations.jl")

include("value.jl")


end


