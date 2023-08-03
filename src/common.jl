# Actions
const actiondir = Dict(:left=>SVector(-1,0), :right=>SVector(1,0), :up=>SVector(0, 1), :down=>SVector(0,-1), :stay=>SVector(0,0))
const actionind = Dict(:left=>1, :right=>2, :up=>3, :down=>4, :stay=>5)
const actionvals = values(actiondir)


# Observations
const OBSERVATIONS = [BitVector([0,0,0,0,0]), 
                    BitVector([1,0,0,0,0]), 
                    BitVector([0,1,0,0,0]), 
                    BitVector([0,0,1,0,0]), 
                    BitVector([0,0,0,1,0]), 
                    BitVector([0,0,0,0,1])]
const obsind = Dict(OBSERVATIONS .=> 1:6)

function targetdir(sp)
    if (sp.robot[1]-sp.target[1]) == 1 # target left of robot
        return :left
    elseif (sp.robot[1]-sp.target[1]) == -1 # target right of robot
        return :right
    elseif (sp.robot[2]-sp.target[2]) == 1 # target below robot
        return :down
    elseif (sp.robot[2]-sp.target[2]) == -1 # target above robot
        return :up
    else
        error("Invalid state")
    end
end

# Rendering
function cell_ctx(xy, size)
    nx, ny = size
    x, y = xy
    return context((x-1)/nx, (ny-y)/ny, 1/nx, 1/ny)
end

sleep_until(t) = sleep(max(t-time(), 0.0))

#@withkw struct problem
#    probtype::String
#end

#function problem(;problem="beliefbias")
    
#    return problem()
#end