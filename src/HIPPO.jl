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
    TSPOMDPBasic,
    TSStateBasic,

    TargetSearchPOMDP,
    TSState

include("basic/core_vanilla.jl")
include("basic/functions_vanilla.jl")
include("basic/observations_vanilla.jl")

include("core.jl")
include("functions.jl")
include("observations.jl")


end


