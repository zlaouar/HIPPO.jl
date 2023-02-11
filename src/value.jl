struct BFORollout # fully observable rollout
    solver::Union{POMDPs.Solver,POMDPs.Policy}
end

function BasicPOMCP.estimate_value(estimator::BasicPOMCP.SolvedFORollout, pomdp::POMDPs.POMDP, start_state, h::BasicPOMCP.BeliefNode, steps::Int)
    newS = TSStateBasic(start_state.robot, start_state.target)
    BasicPOMCP.rollout(estimator, pomdp, newS, h, steps)
end

function BasicPOMCP.convert_estimator(est::BFORollout, solver, pomdp)
    policy = MCTS.convert_to_policy(est.solver, pomdp)
    BasicPOMCP.SolvedFORollout(policy, solver.rng)
end

function POMDPs.simulate(sim::RolloutSimulator, m::POMDP, policy::Policy, initialstate)
    POMDPs.simulate(sim, policy.mdp, policy, initialstate)
end


#Base.convert(::Type{TSState}, a) = B