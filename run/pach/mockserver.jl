using WebSockets: readguarded, open
using JSON

open("ws://127.0.0.1:8084") do ws_client
    println("Android sending waypoint reached")
    #data, success = readguarded(ws_client)
    response = JSON.json(Dict("action" => "FlightParams",
      "args" => Dict("mode" => "waypoint",
                     "altitudeCeiling" => 100,
                     "maxSpeed" => 2,
                     "homeLocation" => [37.7749, -122.4194])))
    write(ws_client, response)
end