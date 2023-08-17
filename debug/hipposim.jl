using HIPPO
using POMDPs
using BasicPOMCP
using POMDPTools
using DiscreteValueIteration
using Profile
using ParticleFilters
using D3Trees
using WebSockets: readguarded, open
using JSON


function generate_path(data, ws_client)
    rewarddist = hcat(data["gridRewards"]...)
    rewarddist = rewarddist .+ abs(minimum(rewarddist)) .+ 0.01

    display(rewarddist)

    mapsize = reverse(size(rewarddist)) # (x,y)
    sinit = RewardState([1, 1], mapsize, vec(trues(mapsize)))#rand(initialstate(msim))
    msolve = RewardPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    solver = POMCPSolver(tree_queries=1000, max_time=0.2, c=80)
    b0 = initialstate(msolve)
    N = 1000
    particle_up = BootstrapFilter(msolve, N)
    particle_b = initialize_belief(particle_up, b0)


    planner = solve(solver, msolve)

    pachSim = PachSimulator(msolve, planner, particle_up, particle_b, sinit)


    location_dict = data["locationDict"]

    locvec, b, sinit = predicted_path(pachSim)
    @info locvec
    pachSim.sinit = sinit
    pachSim.b = b

    response = [location_dict[locvec[i]] for i in eachindex(locvec)]
    println("Sending path: ", response)
    write(ws_client, JSON.json(Dict("action" => "ReturnPath", "args" => Dict("flightPath" => response))))
end

function initialize()
    mapsize = reverse(size(rewarddist)) # (x,y)
    sinit = RewardState([1, 1], mapsize, vec(trues(mapsize)))#rand(initialstate(msim))
    msolve = RewardPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    solver = POMCPSolver(tree_queries=1000, max_time=0.2, c=80)
    b0 = initialstate(msolve)
    N = 1000
    particle_up = BootstrapFilter(msolve, N)
    particle_b = initialize_belief(particle_up, b0)


    planner = solve(solver, msolve)

    pachSim = PachSimulator(msolve, planner, particle_up, particle_b, sinit)

    return pachSim
end

function main()
    println("Opening port")
    #pachSim = initialize()
    while true
        open("ws://127.0.0.1:8082") do ws_client
            print("awaiting data...")
            data, success = readguarded(ws_client)
            if success
                # Parse data as JSON {serviceName, args}
                payload = JSON.parse(String(data))
                action = payload["action"]
                arguments = payload["args"]

                println("Executing Action: ", action)

                if action == "CalculatePath"
                    generate_path(arguments, ws_client)
                end
            end
        end
    end

end

main()


