using WebSockets: readguarded, open
using JSON
using Pkg

Pkg.activate(".")

open("ws://127.0.0.1:8083") do ws_client
    println("Rinao sending rewards")
    #data, success = readguarded(ws_client)
    response = JSON.json(Dict("action" => "ReturnReward",
      "args" => Dict("score" => "0.1")))
    write(ws_client, response)
end