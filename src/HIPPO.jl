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
using StatsBase: sample, Weights
using JLD2

include("common.jl")

include(joinpath(@__DIR__,"camera.jl"))
include(joinpath(@__DIR__,"pachPOMDP","core.jl"))
include(joinpath(@__DIR__,"pachPOMDP","reactive_model.jl"))
include(joinpath(@__DIR__,"pachPOMDP","unified_model.jl"))

export BasicPOMDP,
       RewardPOMDP, 
       BatteryPOMDP, 
       FullPOMDP,
       BasicState,
       RewardState, 
       BatteryState,
       FullState,
       UnifiedState,
       UnifiedPOMDP,
       create_target_search_pomdp

include(joinpath(@__DIR__,"pachPOMDP","simulate.jl"))
export remove_rewards,
       simulate,
       predicted_path,
       HIPPOSimulator, 
       PachSimulator,
       find_closest_grid_point,
       mat_to_inertial_inds

include(joinpath(@__DIR__,"baseline","rawinputs","simulate.jl"))
include(joinpath(@__DIR__,"baseline","rawinputs","action.jl"))
include(joinpath(@__DIR__,"baseline","rawinputs","data.jl"))
include(joinpath(@__DIR__,"baseline","rawinputs","truth.jl"))
include(joinpath(@__DIR__,"baseline","greedy","simulate.jl"))
include(joinpath(@__DIR__,"baseline","greedy","action.jl"))
include(joinpath(@__DIR__,"baseline","map","simulate.jl"))
include(joinpath(@__DIR__,"baseline","map","action.jl"))

export simulate,
       BaselineSimulator,
       UnifiedBaselineSimulator,
       MapBaselineSimulator

#include(joinpath(@__DIR__,"basicPOMDP","functions_vanilla.jl"))

include(joinpath(@__DIR__,"helper.jl"))
export GreedyPolicy,
       FixedPolicy, 
       TargetSearchMDPPolicy, 
       next_action

include("visualize.jl")
export rendhist

include(joinpath(@__DIR__,"XAI","waypoints.jl"))
export get_children, get_children_from_node

include("simutils.jl")
export TargetSearchSim,
       benchmark_planner,
       reset_pomdp!, 
       results, 
       show_benchmark_results,
       results_avg

end


