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

#include(joinpath(@__DIR__,"batteryPOMDP","core_battery.jl"))
#export TSPOMDPBattery, TSStateBattery
#include(joinpath(@__DIR__,"batteryPOMDP","functions_battery.jl"))
#include(joinpath(@__DIR__,"batteryPOMDP","observations_battery.jl"))
#include(joinpath(@__DIR__,"batteryPOMDP","simulate.jl"))


#include(joinpath(@__DIR__,"advPOMDP","core.jl"))
#export TargetSearchPOMDP, TSState
#include(joinpath(@__DIR__,"advPOMDP","functions.jl"))
#include(joinpath(@__DIR__,"advPOMDP","observations.jl")) 


#= include(joinpath(@__DIR__,"metaPOMDP","core.jl"))
include(joinpath(@__DIR__,"metaPOMDP","functions.jl"))
include(joinpath(@__DIR__,"metaPOMDP","observations.jl")) 
 =#
include(joinpath(@__DIR__,"pachPOMDP","core.jl"))
include(joinpath(@__DIR__,"pachPOMDP","functions.jl"))
include(joinpath(@__DIR__,"pachPOMDP","observations.jl")) 
export BasicPOMDP,
       RewardPOMDP, 
       BatteryPOMDP, 
       FullPOMDP,
       BasicState,
       RewardState, 
       BatteryState,
       FullState,
       create_target_search_pomdp

include(joinpath(@__DIR__,"pachPOMDP","simulate.jl"))
# include(joinpath(@__DIR__,"metaPOMDP","simulate.jl"))
export remove_rewards,
       simulateHIPPO, 
       predicted_path,
       HIPPOSimulator, 
       PachSimulator,
       find_closest_grid_point,
       mat_to_inertial_inds

#= include(joinpath(@__DIR__,"baseline","simulate.jl"))
include(joinpath(@__DIR__,"baseline","action.jl"))
include(joinpath(@__DIR__,"baseline","data.jl"))
include(joinpath(@__DIR__,"baseline","truth.jl"))

export simulateBaseline,
       BaselineSimulator =#
#include(joinpath(@__DIR__,"basicPOMDP","core_vanilla.jl"))
#export TSPOMDPBasic, TSStateBasic
include(joinpath(@__DIR__,"basicPOMDP","functions_vanilla.jl"))
#include(joinpath(@__DIR__,"basicPOMDP","observations_vanilla.jl"))


include(joinpath(@__DIR__,"helper.jl"))
export FixedPolicy, TargetSearchMDPPolicy, next_action

include("visualize.jl")
export rendhist
#include(joinpath(@__DIR__,"value.jl"))
#export 
#    BFORollout,
#    SolvedBFORollout,
#    estimate_value,
#    convert_estimator,
#    simulate

include(joinpath(@__DIR__,"XAI","waypoints.jl"))
export get_children, get_children_from_node

end


