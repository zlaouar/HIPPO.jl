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


fixedpolicy(s) = :up

# Function to just print whatever is passed
function generate_path(data, ws_client, planner, msolve, particle_b, particle_up, sinit)
    global rewarddist = hcat(data["gridRewards"]...)
    rewarddist = rewarddist .+ abs(minimum(rewarddist)) .+ 0.01

    
    global locvec, particle_b, sinit = predicted_path(msolve, planner, particle_up, particle_b, sinit)
    response = [location_dict[locvec[i]] for i in eachindex(locvec)]
    println("Sending path: ", response)
    write(ws_client, """{"action": "ReturnPath", "args": "$response"}""")
end
  
function initialize()
    mapsize = reverse(size(rewarddist)) # (x,y)
    global sinit = TSState([1,1], mapsize, vec(trues(mapsize)))#rand(initialstate(msim))
    global msolve = TargetSearchPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    solver = POMCPSolver(tree_queries=1000, max_time=0.2, c=80)
    b0 = initialstate(msolve)
    N = 1000
    particle_up = BootstrapFilter(msolve, N)
    particle_b = initialize_belief(particle_up, b0)
    

    global planner = solve(solver,msolve)

    return planner, msolve, particle_b, particle_up, sinit
end

function main()
    println("Opening port")
    planner, msolve, particle_b, particle_up, sinit = initialize()
    while true
        open("ws://127.0.0.1:8082") do ws_client
            print("something")
            data, success = readguarded(ws_client)
            if success
                # Parse data as JSON {serviceName, args}
                payload = JSON.parse(String(data))
                #print(payload)
                # Access the payload.serviceName
                action = payload["action"]
                arguments = payload["args"]

                println("Executing Action: ",action)
                global location_dict = data["locationDict"]
                println("rmat type: ", typeof(rewarddist))
                println("rmat element type: ", typeof(rewarddist[1]))
                # Decide which function to call based on serviceName
                if action=="CalculatePath"
                    generate_path(arguments,ws_client, planner, msolve, particle_b, particle_up, sinit )
                end
            end
        end
    end

end

main()


