include("hw6_functions.jl")


function main()
    m = TigerPOMDP()

    qmdp_p = qmdp_solve(m)
    action =  POMDPs.action(qmdp_p, DiscreteBelief(m, [0.0;1.0]))
    sarsop_p = solve(SARSOPSolver(), m)
    up = HW6Updater(m)
    #up = DiscreteUpdater(m)

    @show mean(simulate(RolloutSimulator(max_steps=500), m, qmdp_p, up) for _ in 1:5000)
    @show mean(simulate(RolloutSimulator(max_steps=500), m, sarsop_p, up) for _ in 1:5000)

    return nothing
end

main()