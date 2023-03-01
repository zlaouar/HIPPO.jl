using WebSockets: readguarded, open
using JSON


# Function to just print whatever is passed
function test_function(data, ws_client)
    println("Received: ", data)
    response = "received"
    println("Sending back: ", response)
    write(ws_client, """{"action": "rinaoToHippo", "args": "$response"}""")
end
  
open("ws://127.0.0.1:8083") do ws_client
    data, success = readguarded(ws_client)
    if success
        # Parse data as JSON {serviceName, args}
        payload = JSON.parse(String(data))
        # Access the payload.serviceName
        action = payload["action"]
        arguments = payload["args"]

        # Decide which function to call based on serviceName
        router = Dict("action1" => test_function, "action2" => test_function)

        router[action](arguments, ws_client)
    end
end
