using WebSockets: readguarded, open
using JSON
using Pkg

Pkg.activate(".")

function investigate_at_waypoint()
  open("ws://127.0.0.1:8082") do ws_client
      println("HIPPO sending investigate action")
      #data, success = readguarded(ws_client)
      response = JSON.json(Dict("action" => "Investigate",
                                "args" => Dict("event" => "gather-info",
                                                "latitude" => 40.010995,
                                                "longitude" => -105.243904,
                                                "altitude" => 1600,
                                                "speed" => 5.0,
                                                "waypointID" => 1,
                                                "plannerAction" => "[1,1]",
                                                "dwellTime" => 5000.0)))
      write(ws_client, response)
  end
end

function investigate()
  open("ws://127.0.0.1:8082") do ws_client
    println("HIPPO sending investigate action")
    #data, success = readguarded(ws_client)
    response = JSON.json(Dict("action" => "Investigate",
      "args" => Dict("event" => "gather-info")))
    write(ws_client, response)
  end
end

investigate_at_waypoint()