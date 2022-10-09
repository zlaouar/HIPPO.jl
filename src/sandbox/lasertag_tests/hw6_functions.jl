using POMDPs
using DMUStudent.HW6
using POMDPModelTools: transition_matrices, reward_vectors, SparseCat, Deterministic
using QuickPOMDPs: QuickPOMDP
using POMDPModels: TigerPOMDP
using POMDPSimulators: RolloutSimulator
using BeliefUpdaters: DiscreteBelief, DiscreteUpdater
using SARSOP: SARSOPSolver
using POMDPTesting: has_consistent_distributions
using POMDPPolicies: FunctionPolicy
using StatsBase
using LinearAlgebra
##################
# Problem 1: Tiger
##################

#--------
# Updater
#--------
# Note: you can access the transition and observation probabilities through the POMDPs.transtion and POMDPs.observation, and query individual probabilities with the pdf function. For example
# Z(o | a, s') can be programmed with
Z(m::POMDP, a, sp, o) = pdf(observation(m, a, sp), o)
# T(s' | s, a) can be programmed with
T(m::POMDP, s, a, sp) = pdf(transition(m, s, a), sp)
# POMDPs.transtion and POMDPs.observation return distribution objects. See the POMDPs.jl documentation for more details.

struct HW6Updater{M<:POMDP} <: Updater
    m::M
end

function POMDPs.update(up::HW6Updater, b::DiscreteBelief, a, o)
    b_vec = b.state_list
    b_prob = b.b
    bp_vec = zeros(length(states(up.m)))
    #new_p = zeros(length(states(up.m)))
    #bp_vec[1] = 1.0

    for (i,sp) in enumerate(b_vec)
        bp_vec[i] = Z(up.m, a, sp, o)*sum(T(up.m, s, a, sp)*b_prob[j] for (j,s) in enumerate(b_vec))
    end
    bp_vec ./= sum(bp_vec)
    # Fill in code for belief update
    #=
    # Sampling
    num_particles = 100
    for i in num_particles
        bp_vec[i] = rand(transition(up.m, b_vec[i], a)) 
        w_vec[i] = Z(up, a, bp_vec[i], o)
    end

    StatsBase.alias_sample!(bp_vec, w_vec, new_p)

    # Weighting

    # Resampling
    =#


    return DiscreteBelief(up.m, b.state_list, bp_vec)
end



# This is needed to automatically turn any distribution into a discrete belief.
function POMDPs.initialize_belief(up::HW6Updater, distribution::Any)
    b_vec = zeros(length(states(up.m)))
    for s in states(up.m)
        b_vec[stateindex(up.m, s)] = pdf(distribution, s)
    end
    return DiscreteBelief(up.m, b_vec)
end

# Note: to check your belief updater code, you can use BeliefUpdaters: DiscreteUpdater. It should function exactly like your updater.

#-------
# Policy
#-------

struct HW6AlphaVectorPolicy{A} <: Policy
    alphas::Vector{Vector{Float64}}
    alpha_actions::Vector{A}
end

beliefvec(b::DiscreteBelief) = b.b # this function may be helpful to get the belief as a vector in stateindex order

function POMDPs.action(p::HW6AlphaVectorPolicy, b::DiscreteBelief)

    # Fill in code to choose action based on alpha vectors
    bVec = b.b
    alpha_vecs = p.alphas
    acts = p.alpha_actions

    return acts[findmax(hcat(alpha_vecs...)'*bVec)[2]]
end



#------
# QMDP
#------

function qmdp_solve(m, discount=discount(m))

    # Find optimal value function]
    V = [0.0 for s=1:length(states(m))]
    Vprime = [1.0 for s=1:length(states(m))]
    Q = zeros(length(V),length(actions(m)))
    T = transition_matrices(m, sparse=true)
    R = reward_vectors(m)
    γ = discount
    while abs(norm(V)-norm(Vprime)) > 1e-6
        Vprime = V
        for (i,a) in enumerate(actions(m))
            Q[:,i] = R[a] + γ*T[a]*V
        end
        V = maximum(Q, dims=2)
    end
    

    # Fill in VI code for QMDP

    acts = actiontype(m)[]
    alphas = Vector{Float64}[] 
    for a in actions(m)
        push!(alphas, vec(R[a] + γ*T[a]*V))
        push!(acts, a)
        # Fill in alpha vector calculation
    end
    return HW6AlphaVectorPolicy(alphas, acts)
end

function qmdp_solve3(m, discount=discount(m))

    # Find optimal value function]
    V = [0.0 for s=1:length(states(m))]
    Vprime = [1.0 for s=1:length(states(m))]
    Q = zeros(length(V),length(actions(m)))
    T = transition_matrices(m, sparse=true)
    R = reward_vectors(m)
    γ = discount
    #Value Iteration
    while abs(norm(V)-norm(Vprime)) > 1e-6
        Vprime = V
        for (i,a) in enumerate(actions(m))
            Q[:,i] = R[a] + γ*T[a]*V
        end
        V = maximum(Q, dims=2)
    end
    

    # Fill in VI code for QMDP

    acts = actiontype(m)[]
    alphas = Vector{Float64}[] 
    for a in actions(m)
        push!(alphas, vec(R[a] + γ*T[a]*V))
        push!(acts, a)
        # Fill in alpha vector calculation
    end
    return HW6AlphaVectorPolicy(alphas, acts)
end

#=
function main()
    m = TigerPOMDP()

    qmdp_p = qmdp_solve(m)
    action =  POMDPs.action(qmdp_p, DiscreteBelief(m, [0.0;1.0]))
    sarsop_p = solve(SARSOPSolver(), m)
    up = HW6Updater(m)
    #up = DiscreteUpdater(m)

    @show mean(simulate(RolloutSimulator(max_steps=500), m, qmdp_p, up) for _ in 1:5000)
    @show mean(simulate(RolloutSimulator(max_steps=500), m, sarsop_p, up) for _ in 1:5000)

end

main()
=#