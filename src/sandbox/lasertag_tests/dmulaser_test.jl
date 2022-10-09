using POMDPs
using DMUStudent.HW6
using QMDP
using POMDPTools
using LinearAlgebra

m = LaserTagPOMDP()
mtiger = TigerPOMDP()
solver = QMDPSolver(max_iterations=20,
                    belres=1e-3,
                    verbose=true
                   )

#policy = solve(solver, m)
up = DiscreteUpdater(m)

struct HW6AlphaVectorPolicy{A} <: Policy
    alphas::Vector{Vector{Float64}}
    alpha_actions::Vector{A}
end
function POMDPs.action(p::HW6AlphaVectorPolicy, b::DiscreteBelief)

    # Fill in code to choose action based on alpha vectors
    bVec = b.b
    alpha_vecs = p.alphas
    acts = p.alpha_actions

    return acts[findmax(hcat(alpha_vecs...)'*bVec)[2]]
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

#qmdp_p = qmdp_solve3(m)
heuristic = FunctionPolicy(function (b)
            #display(b.b)
            if rand() < 0.4
                return :measure
            end
            # Fill in your heuristic policy here
            return action(qmdp_p,b)
            #return :wait
        end
        )

simulate(RolloutSimulator(), m, heuristic, up)

