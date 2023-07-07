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
using Compose:rectangle,stroke,circle
using BasicPOMCP
using MCTS
using DiscreteValueIteration
using Reel, Cairo, Fontconfig
using AbstractTrees
using D3Trees
using ParticleFilters
using ColorSchemes


include(joinpath(@__DIR__,"common.jl"))

include(joinpath(@__DIR__,"batteryPOMDP","core_battery.jl"))
export TSPOMDPBattery, TSStateBattery
include(joinpath(@__DIR__,"batteryPOMDP","functions_battery.jl"))
include(joinpath(@__DIR__,"batteryPOMDP","observations_battery.jl"))
include(joinpath(@__DIR__,"batteryPOMDP","simulate.jl"))


include(joinpath(@__DIR__,"basicPOMDP","core_vanilla.jl"))
export TSPOMDPBasic, TSStateBasic
include(joinpath(@__DIR__,"basicPOMDP","functions_vanilla.jl"))
include(joinpath(@__DIR__,"basicPOMDP","observations_vanilla.jl"))
include(joinpath(@__DIR__,"basicPOMDP","visualize.jl"))
export GridWorldEnv, renderMDP


include(joinpath(@__DIR__,"advPOMDP","core.jl"))
export TargetSearchPOMDP, TSState
include(joinpath(@__DIR__,"advPOMDP","functions.jl"))
include(joinpath(@__DIR__,"advPOMDP","observations.jl")) 
include(joinpath(@__DIR__,"advPOMDP","simulate.jl"))
export simulateHIPPO, predicted_path, HIPPOSimulator

include(joinpath(@__DIR__,"helper.jl"))
export FixedPolicy, TargetSearchMDPPolicy

#include(joinpath(@__DIR__,"value.jl"))
#export 
#    BFORollout,
#    SolvedBFORollout,
#    estimate_value,
#    convert_estimator,
#    simulate


end


