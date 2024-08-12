using WebSockets: readguarded, open
using JSON

open("ws://127.0.0.1:8084") do ws_client
    while true
      println("Android sending waypoint reached")
      #data, success = readguarded(ws_client)
      response = JSON.json(Dict("action" => "NewFlightStatus",
        "args" => Dict("event" => "waypoint-reached")))
      write(ws_client, response)
      sleep(2)
    end
end