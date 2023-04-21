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
using Reel, Cairo, Fontconfig

export 
    TSPOMDPBasic,
    TSStateBasic,

    TargetSearchPOMDP,
    TSState,

    BFORollout,
    SolvedBFORollout,
    estimate_value,
    convert_estimator,
    simulate,

    customsim,

    FixedPolicy,
    TargetSearchMDPPolicy

include(joinpath(@__DIR__,"common.jl"))

include(joinpath(@__DIR__,"basicPOMDP","core_vanilla.jl"))
include(joinpath(@__DIR__,"basicPOMDP","functions_vanilla.jl"))
include(joinpath(@__DIR__,"basicPOMDP","observations_vanilla.jl"))

include(joinpath(@__DIR__,"advPOMDP","core.jl"))
include(joinpath(@__DIR__,"advPOMDP","functions.jl"))
include(joinpath(@__DIR__,"advPOMDP","observations.jl"))
include(joinpath(@__DIR__,"advPOMDP", "helper.jl"))

include("value.jl")


end


