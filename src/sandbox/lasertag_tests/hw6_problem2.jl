include("hw6_functions.jl")

function problem2()
    
    cancer = QuickPOMDP(

        states = [:healthy,:in_situ_cancer, :invasive_cancer, :death],
        actions = [:wait, :test, :treat],
        observations = [true, false],

        transition = function (s, a)
            if s == :healthy
                return SparseCat([:in_situ_cancer,s], [0.02,0.98])
            elseif s == :in_situ_cancer && a == :treat
                return SparseCat([:healthy,s], [0.60,0.40])
            elseif s == :in_situ_cancer && a != :treat
                return SparseCat([:invasive_cancer,s], [0.10,0.90])
            elseif s == :invasive_cancer && a == :treat
                return SparseCat([:healthy,:death,s], [0.20,0.20,0.6])
            elseif s == :invasive_cancer && a != :treat
                return SparseCat([:death,s], [0.60,0.40])
            else 
                return Deterministic(s)
            end
        end,

        observation = function (a, sp)
            if a == :test
                if sp == :healthy
                    return SparseCat([true, false], [0.05, 0.95])
                elseif sp == :in_situ_cancer
                    return SparseCat([true, false], [0.80, 0.20])
                else
                    return Deterministic(false)
                end
            elseif a == :treat
                return Deterministic(true)
            else
                return Deterministic(false)
            end
        end,

        reward = function (s, a)
            if a == :wait
                if s == :death
                    return 0.0
                else
                    return 1.0
                end
            elseif a == :test
                if s == :death
                    return 0.0
                else
                    return 0.8
                end
            else # a = :treat
                if s == :death
                    return 0.0
                else
                    return 0.1
                end
            end
        end,

        initialstate = Deterministic(:healthy),

        discount = 0.99,
        isterminal = s->s==:death,
        
    )
    
    
    @assert has_consistent_distributions(cancer)

    qmdp_p = qmdp_solve(cancer)
    sarsop_solver = SARSOPSolver()
    sarsop_solver.verbose = false
    sarsop_p = solve(sarsop_solver, cancer)
    up = HW6Updater(cancer)
    #up2 = DiscreteUpdater(cancer)

    heuristic = FunctionPolicy(function (b)
                                if rand() < 0.3
                                    return :test
                                end

                                return action(qmdp_p,b)
                            end
                            )

    @show mean(simulate(RolloutSimulator(), cancer, qmdp_p, up) for _ in 1:1000)     # Should be approximately 66
    @show mean(simulate(RolloutSimulator(), cancer, heuristic, up) for _ in 1:1000)
    @show mean(simulate(RolloutSimulator(), cancer, sarsop_p, up) for _ in 1:1000)   # Should be approximately 79

    return nothing
end

problem2()