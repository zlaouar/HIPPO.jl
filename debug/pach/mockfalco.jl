using WebSockets: readguarded, open
using JSON
using Pkg

Pkg.activate(".")

open("ws://127.0.0.1:8085") do ws_client
    println("Android sending waypoint reached")
    #data, success = readguarded(ws_client)
    response = JSON.json(Dict("action" => "ConfidenceScore",
      "args" => Dict("score" => 0.7)))
    write(ws_client, response)
end