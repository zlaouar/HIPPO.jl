using Pkg

Pkg.activate(".")

using HIPPO

Pkg.activate("test")

using BenchmarkTools
using StaticArrays

# First, let's define both versions of our functions:
function generate_fov_func(state, orientation)
    # TODO: create fov as a function of orientation, altitude, and camera angle
    states = []
    # diff = orientdir(s.orientation)
    for i in s.robot[1]-2:s.robot[1]+2
        for j in s.robot[2]-2:s.robot[2]+2
            push!(states, bounce(m, state, SVector(i, j)))
        end
    end
    return unique(states)
end

# Vector version
function precompute_fov_vector(generate_fov_func, grid_size, orientations)
    fov_lookup = Dict{Tuple{Int, Int, Symbol}, Vector{Tuple{Int, Int}}}()
    for x in 1:grid_size[1], y in 1:grid_size[2], orientation in orientations
        state = (x, y)
        fov = generate_fov_func(state, orientation)
        fov_lookup[(x, y, orientation)] = fov
    end
    return fov_lookup
end

function is_in_fov_vector(cell, robot_state, robot_orientation, fov_lookup)
    fov_vector = fov_lookup[(robot_state[1], robot_state[2], robot_orientation)]
    return cell in fov_vector
end

# Set version
function precompute_fov_set(generate_fov_func, grid_size, orientations)
    fov_lookup = Dict{Tuple{Int, Int, Symbol}, Set{Tuple{Int, Int}}}()
    for x in 1:grid_size[1], y in 1:grid_size[2], orientation in orientations
        state = (x, y)
        fov = generate_fov_func(state, orientation)
        fov_lookup[(x, y, orientation)] = Set(fov)
    end
    return fov_lookup
end

function is_in_fov_set(cell, robot_state, robot_orientation, fov_lookup)
    fov_set = fov_lookup[(robot_state[1], robot_state[2], robot_orientation)]
    return cell in fov_set
end

# Now, let's create a simple generate_fov function for testing:
function generate_fov(state, orientation)
    # This is a dummy function that returns a small field of view
    # You should replace this with your actual generate_fov function
    return [(state[1]+i, state[2]+i) for i in 1:5]
end

# Set up test parameters
grid_size = (10, 10)
orientations = [:north, :east, :south, :west]

# Precompute FOV lookups
fov_lookup_vector = precompute_fov_vector(generate_fov, grid_size, orientations)
fov_lookup_set = precompute_fov_set(generate_fov, grid_size, orientations)

# Benchmark precomputation
println("Precomputation benchmark:")
@btime precompute_fov_vector($generate_fov, $grid_size, $orientations)
@btime precompute_fov_set($generate_fov, $grid_size, $orientations)

# Benchmark lookups
println("\nLookup benchmark:")
robot_state = (5, 5)
robot_orientation = :north
cell_to_check = (6, 6)

@btime is_in_fov_vector($cell_to_check, $robot_state, $robot_orientation, $fov_lookup_vector)
@btime is_in_fov_set($cell_to_check, $robot_state, $robot_orientation, $fov_lookup_set)

# Benchmark multiple lookups
println("\nMultiple lookups benchmark:")
function multiple_lookups_vector(n, fov_lookup)
    count = 0
    for _ in 1:n
        x, y = rand(1:10), rand(1:10)
        orientation = rand(orientations)
        cell = (rand(1:10), rand(1:10))
        if is_in_fov_vector(cell, (x, y), orientation, fov_lookup)
            count += 1
        end
    end
    return count
end

function multiple_lookups_set(n, fov_lookup)
    count = 0
    for _ in 1:n
        x, y = rand(1:10), rand(1:10)
        orientation = rand(orientations)
        cell = (rand(1:10), rand(1:10))
        if is_in_fov_set(cell, (x, y), orientation, fov_lookup)
            count += 1
        end
    end
    return count
end

@btime multiple_lookups_vector(10000, $fov_lookup_vector)
@btime multiple_lookups_set(10000, $fov_lookup_set)