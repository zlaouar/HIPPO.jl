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
    # change pomdp reward mat to updated POMDP reward mat 
    #pomdp.reward = rewarddist 
    BasicPOMCP.rollout(newEst, pomdp, newS, h, steps)
end

function BasicPOMCP.convert_estimator(est::BFORollout, solver, pomdp)
    policy = MCTS.convert_to_policy(est.solver, pomdp)
    SolvedBFORollout(policy, solver.rng)
end

#function POMDPs.simulate(sim::RolloutSimulator, m::POMDP, policy::ValueIterationPolicy, initialstate)
#    POMDPs.simulate(sim, policy.mdp, policy, initialstate)
#end

function DiscreteValueIteration.action(policy::ValueIterationPolicy, s)
    # get action corresponding to current state from mdp policy but calculate reward from POMDP
    newS = TSStateBasic(s.robot, s.target)
    sidx = stateindex(policy.mdp, newS)
    aidx = policy.policy[sidx]
    return policy.action_map[aidx]
end
#Base.convert(::Type{TSState}, a) = B