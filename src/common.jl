# Actions
const actiondir = Dict(:left=>SVector(-1,0), :right=>SVector(1,0),
                       :up=>SVector(0, 1), :down=>SVector(0,-1), :stay=>SVector(0,0),
                       :nw=>SVector(-1,1), :ne=>SVector(1,1), 
                       :sw=>SVector(-1,-1), :se=>SVector(1,-1))

const actionind = Dict(:left=>1, :right=>2, :up=>3, :down=>4, :stay=>5,
                       :nw=>6, :ne=>7, :sw=>8, :se=>9)
const actionvals = values(actiondir)

# Camera Characteristics
const ORIENTATIONS = [:left, :right, :up, :down, :nw, :ne, :sw, :se]

orientdir = Dict{Vector{Int}, Symbol}(
    [-1, 0] => :left,
    [1, 0] => :right,
    [0, 1] => :up,
    [0, -1] => :down,
    [0, 0] => :stay,
    [-1, 1] => :nw,
    [1, 1] => :ne,
    [-1, -1] => :sw,
    [1, -1] => :se
)

HEADING_VALS = (45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0, 0.0)
# headingdir = Dict{Float64, Symbol}(
#     45.0 => :ne,
#     90.0 => :up,
#     135.0 => :nw,
#     180.0 => :left,
#     225.0 => :sw,
#     270.0 => :down,
#     315.0 => :se,
#     0.0 => :right
# )

headingdir = Dict{Symbol, Float64}(
    :ne => 45.0,
    :up => 90.0,
    :nw => 135.0,
    :left => 180.0,
    :sw => 225.0,
    :down => 270.0,
    :se => 315.0,
    :right => 0.0
)

# Observations
#= const OBSERVATIONS = [BitVector([0,0,0,0,0]), 
                    BitVector([1,0,0,0,0]), 
                    BitVector([0,1,0,0,0]), 
                    BitVector([0,0,1,0,0]), 
                    BitVector([0,0,0,1,0]), 
                    BitVector([0,0,0,0,1])] =#

const OBSERVATIONS = [:next_waypoint, :gather_action, :target_found]
const E2E_OBSERVATIONS = [:low, :medium, :high]
# const obsind = Dict(OBSERVATIONS .=> 1:6)

const obsind = Dict(OBSERVATIONS .=> 1:3)

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
    #return context((x-1)/nx, (ny-y)/ny, 1/max(nx,ny), 1/max(nx,ny))
    return context((x-1)/nx, (ny-y)/ny, 1/nx, 1/ny)
end
function rect_ctx(xy, size, width, height)
    nx, ny = size
    x, y = xy
    return context((x-1)/nx, (ny-y)/ny, width/max(nx,ny), height/max(nx,ny))
    #return context((x-1)/nx, (ny-y)/ny, 1/nx, 1/ny)
end

function coord(xy, size)
    nx, ny = size
    x, y = xy
    return ((x-1)/nx, (ny-y)/ny)
end

sleep_until(t) = sleep(max(t-time(), 0.0))

#@withkw struct problem
#    probtype::String
#end

#function problem(;problem="beliefbias")
    
#    return problem()
#end