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


function generate_predicted_path(data, ws_client)
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

function generate_sim_path(data, ws_client)
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

    location_dict = data["locationDict"]

    hipposim = HIPPOSimulator(msim=msolve, planner=planner, up=particle_up, b=particle_b, 
                            sinit=sinit, max_iter=15)
    @info "Running simulation"
    hist, _, _, _, _ = simulateHIPPO(hipposim)
    a_traj = getfield.(hist, :a)
    locvec = HIPPO.loctostr(HIPPO.generatelocation(msolve, a_traj, sinit.robot))

    #locvec, b, sinit = predicted_path(pachSim)
    @info locvec
    #pachSim.sinit = sinit
    #pachSim.b = b

    response = [location_dict[locvec[i]] for i in eachindex(locvec)]
    println("Sending path: ", response)
    write(ws_client, JSON.json(Dict("action" => "ReturnPath", "args" => Dict("flightPath" => response))))
end

function initialize(rewarddist, location_dict, flightParams)
    mapsize = reverse(size(rewarddist)) # (x,y)
    maxbatt = 100

    #if isnothing(flightParams)
    #    flightParams = HIPPO.FlightParams("waypoint", 100, 20.0, [40.019375, -105.265566])#[37.7749, -122.4194])
    #end

    closest_point = find_closest_grid_point(location_dict, flightParams.home_location)
    initial_point = mat_to_inertial_inds(mapsize, closest_point)
    sinit = FullState(initial_point, mapsize, vec(trues(mapsize)), maxbatt)#rand(initialstate(msim))

    msolve = create_target_search_pomdp(sinit, 
                                    size=mapsize, 
                                    rewarddist=rewarddist, 
                                    maxbatt=maxbatt, options=Dict(:observation_model=>:falco))

    #msolve = RewardPOMDP(sinit, size=mapsize, rewarddist=rewarddist)
    solver = POMCPSolver(tree_queries=1000, max_time=0.2, c=80)
    b0 = initialstate(msolve)
    N = 1000
    particle_up = BootstrapFilter(msolve, N)
    particle_b = initialize_belief(particle_up, b0)

    planner = solve(solver, msolve)
    # default_flight_mode = "waypoint"
    # default_max_speed = 20.0
    # default_home_location = [37.7749, -122.4194]
    # default_action = :nothing
    # default_waypointID = 0
    
    default_action = :up
    default_waypointID = 0
    pachSim = PachSimulator(msolve, planner, particle_up, particle_b, sinit, 
                            location_dict, default_action, default_waypointID,
                            flightParams)
                        
    return pachSim
end

function update_reward(data, ws_client, pachSim, initialized, flightParams)
    rewarddist = hcat(data["gridRewards"]...)
    rewarddist = rewarddist .+ abs(minimum(rewarddist)) .+ 0.01
    location_dict = data["locationDict"]

    if initialized
        pachSim.msim.reward = rewarddist
        pachSim.location_dict = location_dict
        println("reward updated")
    else
        pachSim = initialize(rewarddist, location_dict, flightParams)
        a, _ = BasicPOMCP.action_info(pachSim.planner, pachSim.b)
        pachSim.previous_action = a
        sp, _, _ = @gen(:sp,:o,:r)(pachSim.msim, pachSim.sinit, a)
        loc = HIPPO.loctostr(HIPPO.generatelocation(pachSim.msim, [a], pachSim.sinit.robot))
        @info "s: ", pachSim.sinit.robot, " | sp: ", sp.robot , " | loc: ", loc, " | a: ", a 
        pachSim.sinit = sp

        response = location_dict[loc[1]]
        commanded_alt = response[3] + pachSim.flight_params.desired_agl_alt

        println("Sending first action: ", response)

        write(ws_client, JSON.json(Dict("action" => "NextFlightWaypoint", "args" => Dict("latitude" => response[1],
                                                                                    "longitude" => response[2],
                                                                                    "altitude" => commanded_alt,
                                                                                    "speed" => pachSim.flight_params.max_speed,
                                                                                    "waypointID" => pachSim.waypointID,
                                                                                    "plannerAction" => string(a),
                                                                                    "dwellTime" => 5.0))))
        
        println("pachSim initialized")
        pachSim.waypointID += 1    
    end

    return pachSim
end

function update_params(data)
    if !haskey(data, "homeLocation")
        data["homeLocation"] = [40.019375, -105.265566]#[37.7749, -122.4194]
    end
    flightParams = HIPPO.FlightParams(data["mode"], 
                    data["altitudeCeiling"], 
                    data["maxSpeed"], 
                    data["homeLocation"])
                   # [37.7749, -122.4194]

    return flightParams
end

# 1) Update belief
# 2) Plan for reaching next waypoint
# 3) Send next waypoint


function generate_next_action(data, ws_client, pachSim)
    (msim, up, b, sinit, previous_action) = (pachSim.msim, pachSim.up, pachSim.b, 
                                             pachSim.sinit, pachSim.previous_action)
    println("data: ", data)
    status = data["event"]

    if status == "waypoint-reached"
        o = :next_waypoint
    elseif status == "gather-info"
        o = :gather_action 
    else 
        o = :nothing
    end

    #o = status == "waypoint-reached" ? :waypoint_reached : 
    #    status == "gather-info" ? :gather_info : :nothing

    println("status: ", status)
    println("o: ", o)
    b = update(up, b, previous_action, o)
    pachSim.b = b

    remove_rewards(pachSim.msim, sinit.robot) # remove reward at current state
    #tree, b = conditional_path(pachSim)
    #@warn "rewards: ", pachSim.msim.reward
    hnode = BasicPOMCP.POMCPObsNode(pachSim.planner._tree, 1)
    a = next_action(hnode, previous_action)
    sp, _, _ = @gen(:sp,:o,:r)(pachSim.msim, pachSim.sinit, a)
    loc = HIPPO.loctostr(HIPPO.generatelocation(msim, [a], sinit.robot))
    @info "s: ", pachSim.sinit.robot, " | sp: ", sp.robot , " | loc: ", loc, " | a: ", a, "prev a: ", previous_action
    response = pachSim.location_dict[loc[1]]

    println("Sending waypoint: ", response)
    write(ws_client, JSON.json(Dict("action" => "NextFlightWaypoint", "args" => Dict("latitude" => response[1],
                                                                                    "longitude" => response[2],
                                                                                    "altitude" => response[3],
                                                                                    "speed" => pachSim.flight_params.max_speed,
                                                                                    "waypointID" => pachSim.waypointID,
                                                                                    "plannerAction" => string(a),
                                                                                    "dwellTime" => 5.0))))
    #Plan for reaching next waypoint
    #inchrome(D3Tree(pachSim.planner._tree))
    newa, info = BasicPOMCP.action_info(pachSim.planner, pachSim.b, tree_in_info = true)


    pachSim.previous_action = a
    pachSim.sinit = sp

    return pachSim
end

function main()
    println("HIPPO: Opening port\n")
    pachSim = nothing
    flightParams = nothing
    initialized = false
    #while true
    open("ws://127.0.0.1:8082") do ws_client
        print("HIPPO: awaiting data...\n")
        while !eof(ws_client)
            data, success = readguarded(ws_client)
            if success
                # Parse data as JSON {serviceName, args}
                payload = JSON.parse(String(data))
                action = payload["action"]
                arguments = payload["args"]

                println("Executing Action: ", action)

                if action == "CalculatePath"
                    println("initialized: ", initialized)
                    pachSim = update_reward(arguments, ws_client, pachSim, initialized, flightParams)
                    initialized = true
                    #println("reward updated")
                elseif action == "FlightStatus"
                    pachSim = generate_next_action(arguments, ws_client, pachSim)

                elseif action == "FlightParams"
                    #desired_agl_alt = arguments["altitudeCeiling"]
                    pachSim.flightParams = update_params(arguments)
                    println("Updated Params")
                end
            end
        end
    end
end

main()