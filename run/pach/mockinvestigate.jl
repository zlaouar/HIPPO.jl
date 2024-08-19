using WebSockets: readguarded, open
using JSON
using Pkg

Pkg.activate(".")

function investigate_with_latlon()
  open("ws://127.0.0.1:8082") do ws_client
      println("HIPPO sending investigate action")
      #data, success = readguarded(ws_client)
      response = JSON.json(Dict("action" => "Investigate",
        "args" => Dict("event" => "gather-info", "lat" => 0.0, "lon" => 0.0)))
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

investigate_with_latlon()