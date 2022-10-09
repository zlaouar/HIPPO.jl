# to display what's going on, use the following code
using ElectronDisplay
using POMDPSimulators: stepthrough
using POMDPModelTools: render
include("hw6_functions.jl")

m = LaserTagPOMDP()

qmdp_p = qmdp_solve3(m)
#up = HW6Updater(m)



up = DiscreteUpdater(m) # you may want to replace this with your updater to test it
for step in stepthrough(m, qmdp_p, up, "a,r,sp,o,bp"; max_steps=10)
    electrondisplay(render(m, step))
end

#@show mean(simulate(RolloutSimulator(), m, qmdp_p, up) for _ in 1:1000)
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

@show HW6.evaluate((heuristic, up),"zakariya.laouar@colorado.edu", n_episodes=1000) # 27.44