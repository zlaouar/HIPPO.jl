using Pkg
Pkg.activate(joinpath(@__DIR__,"..",".."))
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
using StaticArrays

struct WaypointParams
    show::Bool
    depth::Int
    n_actions::Int
end

function generate_predicted_path(location_dict, pachSim, ws_client)
    locvec, b, sinit = predicted_path(pachSim; pathlen=15)
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
    solver = POMCPSolver(tree_queries=1000, max_time=0.2, c=80, tree_in_info=true)
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

function get_obstacles(keepout_zones, mapsize)
    keepmat = stack(keepout_zones)
    indices = findall(x -> x == 1, keepmat)
    indtup = Tuple.(indices)
    obstacles = SVector{2}.([mat_to_inertial_inds(mapsize, indtup[i]) for i ∈ eachindex(indtup)])
    return obstacles
end

function initialize(rewarddist, location_dict, keepout_zones, resolution, flightParams)
    mapsize = reverse(size(rewarddist)) # (x,y)
    maxbatt = 1000
    closest_point = find_closest_grid_point(location_dict, flightParams.home_location)
    initial_point = mat_to_inertial_inds(mapsize, closest_point)
    sinit = FullState(initial_point, collect(mapsize) + [1,1], vec(trues(mapsize)), maxbatt)#rand(initialstate(msim))

    # keepmat = stack(keepout_zones)
    # indices = findall(x -> x == 1, keepmat)
    # indtup = Tuple.(indices)
    # obstacles = SVector{2}.([mat_to_inertial_inds(mapsize, indtup[i]) for i ∈ eachindex(indtup)])

    obstacles = get_obstacles(keepout_zones, mapsize)

    msolve = create_target_search_pomdp(sinit, 
                                    size=mapsize, 
                                    rewarddist=rewarddist, 
                                    maxbatt=maxbatt, 
                                    options=Dict(:observation_model=>:falco),
                                    obstacles=obstacles,
                                    resolution=resolution)

    solver = POMCPSolver(tree_queries=50000, max_time=1.0, c=200, tree_in_info=true) #max_depth=3,c=1000,
    b0 = initialstate(msolve)
    N = 1000
    particle_up = BootstrapFilter(msolve, N)
    particle_b = initialize_belief(particle_up, b0)

    planner = solve(solver, msolve)
    
    default_action = :up
    default_waypointID = 0
    pachSim = PachSimulator(msolve, planner, particle_up, particle_b, sinit, 
                            location_dict, default_action, default_waypointID,
                            flightParams)
                        
    return pachSim
end

function update_reward(data, ws_client, pachSim, initialized, flightParams; waypoint_params=WaypointParams(false,0,0))
    rewarddist = hcat(data["gridRewards"]...)
    rewarddist = rewarddist .+ abs(minimum(rewarddist)) .+ 0.01
    location_dict = data["locationDict"]
    keepout_zones = stack(data["keepOutZones"])
    resolution = data["resolution"]


    if initialized
        if flightParams.flight_mode == "path"
            generate_predicted_path(location_dict, pachSim, ws_client)
        end
        pachSim.msim.reward = rewarddist
        pachSim.msim.obstacles = get_obstacles(keepout_zones, reverse(size(rewarddist)))
        pachSim.location_dict = location_dict
        println("reward updated")
    else
        pachSim = initialize(rewarddist, location_dict, keepout_zones, resolution, flightParams)

        if flightParams.flight_mode == "path"
            generate_predicted_path(location_dict, pachSim, ws_client)
            return pachSim
        end

        a, a_info = BasicPOMCP.action_info(pachSim.planner, pachSim.b)
        # inchrome(D3Tree(a_info[:tree]))
        pachSim.previous_action = a
        sp = @gen(:sp)(pachSim.msim, pachSim.sinit, a)
        spro = HIPPO.bounce(pachSim.msim, pachSim.sinit.robot, HIPPO.actiondir[a])
        sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
        loc = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, sp.robot)])
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
                                                                                    "dwellTime" => 5000.0))))
        if waypoint_params.show && flightParams.flight_mode == "waypoint" ##
            tree = a_info[:tree]
            future_nodes,future_opacities,future_parents = get_children(pachSim.msim,pachSim.b,tree;depth=waypoint_params.depth,n_actions=waypoint_params.n_actions)
            dict_list = []
            for i in eachindex(future_nodes)#[2:end]) #Exclude the point already passed as "NextFlightWaypoint"
                lc_str = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, future_nodes[i].robot)])
                lat,lon,_ = location_dict[lc_str[1]]
                par_lc_str = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, future_parents[i].robot)])
                par_lat,par_lon,_ = location_dict[par_lc_str[1]]
                push!(dict_list,Dict("latitude"=>lat,"longitude"=>lon,"opacity"=>future_opacities[i],"par_latitude"=>par_lat,"par_longitude"=>par_lon))
            end
            write(ws_client, JSON.json(Dict("action"=>"FutureWaypoints", "args"=>dict_list)))
            # @info [future_parents[i].robot => future_nodes[i].robot for i in eachindex(future_nodes)]
            # @info future_opacities
            # @info dict_list
        end
        println("pachSim initialized")
        pachSim.waypointID += 1    

        #Plan for reaching next waypoint
        nextwp_belief = update(pachSim.up, pachSim.b, a, :next_waypoint)
        BasicPOMCP.action_info(pachSim.planner, nextwp_belief, tree_in_info = true)
        pachSim.b = nextwp_belief
    end

    return pachSim
end

function update_params(data)
    if !haskey(data, "homeLocation") || isnothing(data["homeLocation"])
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
function best_action(t::BasicPOMCP.POMCPTree)
    h = 1
    best_node = first(t.children[h])
    best_v = t.v[best_node]
    @assert !isnan(best_v)
    for node in t.children[h][2:end]
        if t.v[node] > best_v
            best_v = t.v[node]
            best_node = node
        end
    end

    return t.a_labels[best_node]
end

function generate_next_action(data, ws_client, pachSim, flightParams; waypoint_params=WaypointParams(false,0,0))
    (msim, up, b, sinit, previous_action) = (pachSim.msim, pachSim.up, pachSim.b, 
                                             pachSim.sinit, pachSim.previous_action)
    println("data: ", data)
    status = data["event"]

    if status == "waypoint-reached"
        o = :next_waypoint
    elseif status == "gather-info"
        o = :gather_action 
    else 
        error("Invalid observation")
        # o = :nothing
    end

    println("status: ", status)
    println("o: ", o)
    #b = update(up, b, previous_action, o)
    #pachSim.b = b

    # remove_rewards(pachSim.msim, sinit.robot) # remove reward at current state
    #tree, b = conditional_path(pachSim)
    #@warn "rewards: ", pachSim.msim.reward
    # inchrome(D3Tree(pachSim.planner._tree))

    a = best_action(pachSim.planner._tree)
    
    # hnode = BasicPOMCP.POMCPObsNode(pachSim.planner._tree, 1)
    # a = next_action(hnode, previous_action)
    sp = @gen(:sp)(pachSim.msim, pachSim.sinit, a)
    spro = HIPPO.bounce(pachSim.msim, pachSim.sinit.robot, HIPPO.actiondir[a])
    sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
    loc = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, sp.robot)])
    @info "s: ", pachSim.sinit.robot, " | sp: ", sp.robot , " | loc: ", loc, " | a: ", a, "prev a: ", previous_action, "support(b)[1].robot:" support(pachSim.b)[1].robot
    response = pachSim.location_dict[loc[1]]

    commanded_alt = response[3] + pachSim.flight_params.desired_agl_alt

    println("Sending waypoint: ", response)
    write(ws_client, JSON.json(Dict("action" => "NextFlightWaypoint", "args" => Dict("latitude" => response[1],
                                                                                    "longitude" => response[2],
                                                                                    "altitude" => commanded_alt,
                                                                                    "speed" => pachSim.flight_params.max_speed,
                                                                                    "waypointID" => pachSim.waypointID,
                                                                                    "plannerAction" => string(a),
                                                                                    "dwellTime" => 5000.0))))

    
    #Plan for reaching next waypoint
    nextwp_belief = update(pachSim.up, pachSim.b, a, :next_waypoint)
    BasicPOMCP.action_info(pachSim.planner, nextwp_belief, tree_in_info = true)
    pachSim.b = nextwp_belief
    if waypoint_params.show  && flightParams.flight_mode == "waypoint"##
        #future_nodes,future_opacities,future_parents = get_children_from_node(pachSim.msim,pachSim.b,pachSim.planner._tree,o,pachSim.previous_action;depth=waypoint_params.depth,n_actions=waypoint_params.n_actions)
        future_nodes,future_opacities,future_parents = get_children(pachSim.msim,pachSim.b,pachSim.planner._tree;depth=waypoint_params.depth,n_actions=waypoint_params.n_actions)
        #Add selected action/waypoint from previous tree to points to render
        pushfirst!(future_nodes,sp)
        pushfirst!(future_opacities,maximum(future_opacities))
        pushfirst!(future_parents,pachSim.sinit)

        dict_list = []
        for i in eachindex(future_nodes)#[2:end]) #Exclude the point already passed as "NextFlightWaypoint"
            lc_str = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, future_nodes[i].robot)])
            lat,lon,_ = pachSim.location_dict[lc_str[1]]
            par_lc_str = HIPPO.loctostr([HIPPO.convertinds(pachSim.msim, future_parents[i].robot)])
            par_lat,par_lon,_ = pachSim.location_dict[par_lc_str[1]]
            push!(dict_list,Dict("latitude"=>lat,"longitude"=>lon,"opacity"=>future_opacities[i],"par_latitude"=>par_lat,"par_longitude"=>par_lon))
        end
        write(ws_client, JSON.json(Dict("action"=>"FutureWaypoints", "args"=>dict_list)))
        # @info [node.robot for node in future_nodes]
        # @info [future_parents[i].robot => future_nodes[i].robot for i in eachindex(future_nodes)]
        # @info dict_list
    end

    pachSim.previous_action = a
    pachSim.sinit = sp
    pachSim.waypointID += 1

    return pachSim
end

function main()
    println("HIPPO: Opening port\n")
    pachSim = nothing
    flightParams = nothing
    initialized = false

    show_way = true
    way_depth = 2
    way_actions = 4
    way_params = WaypointParams(show_way,way_depth,way_actions)

    flight_params_updated = false

    open("ws://127.0.0.1:8082") do ws_client
        print("HIPPO: awaiting data...\n")
        while !eof(ws_client)
            data, success = readguarded(ws_client)
            if success
                payload = JSON.parse(String(data))
                action = payload["action"]
                arguments = payload["args"]

                println("Executing Action: ", action)

                if action == "CalculatePath"
                    # @info "In Calc Path"
                    println("initialized: ", initialized)
                    pachSim = update_reward(arguments, ws_client, pachSim, initialized, flightParams; waypoint_params=way_params)
                    initialized = true

                    if flight_params_updated
                        pachSim.flight_params = flightParams
                        flight_params_updated = false
                    end
                    
                elseif action == "FlightStatus"
                    # println("=========================")
                    # @info "In FS"
                    if initialized 
                        pachSim = generate_next_action(arguments, ws_client, pachSim, flightParams; waypoint_params=way_params)
                    else
                        println("FlightStatus: Not initialized, waiting on new params")
                    end

                elseif action == "FlightParams"
                    # @info "In FP"
                    flightParams = update_params(arguments)
                    flight_params_updated = true
                    println("Updated Params")

                elseif action == "Error"
                    println("Error: ", arguments)

                elseif action == "ResetHIPPO"
                    pachSim = nothing
                    flightParams = nothing
                    initialized = false
                    flight_params_updated = false
                    println("HIPPO Reset")
                end
            end
        end
    end
end

main()
