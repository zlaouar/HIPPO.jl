struct BFORollout # fully observable rollout
    solver::Union{POMDPs.Solver,POMDPs.Policy}
end

struct SolvedBFORollout{P<:POMDPs.Policy,RNG<:AbstractRNG}
    policy::P
    rng::RNG
end

function BasicPOMCP.estimate_value(estimator::SolvedBFORollout, pomdp::POMDP, start_state, h::BasicPOMCP.BeliefNode, steps::Int)
    newS = TSStateBasic(start_state.robot, start_state.target)
    newEst = BasicPOMCP.SolvedFORollout(estimator.policy, estimator.rng)
    BasicPOMCP.rollout(newEst, pomdp, newS, h, steps)
end

function BasicPOMCP.convert_estimator(est::BFORollout, solver, pomdp)
    policy = MCTS.convert_to_policy(est.solver, pomdp)
    SolvedBFORollout(policy, solver.rng)
end

function POMDPs.simulate(sim::RolloutSimulator, m::POMDP, policy::ValueIterationPolicy, initialstate)
    POMDPs.simulate(sim, policy.mdp, policy, initialstate)
end


#Base.convert(::Type{TSState}, a) = B