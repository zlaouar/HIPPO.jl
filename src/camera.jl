struct Vector3D
    x::Float64
    y::Float64
    z::Float64
end

Base.:+(a::Vector3D, b::Vector3D) = Vector3D(a.x + b.x, a.y + b.y, a.z + b.z)
Base.:-(a::Vector3D, b::Vector3D) = Vector3D(a.x - b.x, a.y - b.y, a.z - b.z)
Base.:*(a::Vector3D, b::Float64) = Vector3D(a.x * b, a.y * b, a.z * b)
Base.:*(b::Float64, a::Vector3D) = a * b

function normalize(v::Vector3D)
    magnitude = sqrt(v.x^2 + v.y^2 + v.z^2)
    Vector3D(v.x / magnitude, v.y / magnitude, v.z / magnitude)
end

struct CameraInfo
    FOVh::Float64
    FOVv::Float64
    altitude::Float64
    roll::Float64
    pitch::Float64
    heading::Float64
end

function getBoundingPolygon(cam_info::CameraInfo, x::Float64, y::Float64)
    (;FOVh, FOVv, altitude, roll, pitch, heading) = cam_info

    ray1 = ray(FOVh, FOVv, 1, 1)
    ray2 = ray(FOVh, FOVv, 1, -1)
    ray3 = ray(FOVh, FOVv, -1, -1)
    ray4 = ray(FOVh, FOVv, -1, 1)

    rotatedVectors = rotateRays([ray1, ray2, ray3, ray4], roll, pitch, heading)
    
    origin = Vector3D(x, y, altitude)
    intersections = getRayGroundIntersections(rotatedVectors, origin)

    # Sort the intersections in clockwise order
    sorted_intersections = sortClockwise(intersections)

    return sorted_intersections
end

function ray(FOVh::Float64, FOVv::Float64, signV::Int, signH::Int)
    v = Vector3D(signH * tan(FOVh/2), signV * tan(FOVv/2), -1)
    normalize(v)
end

function rotateRays(rays::Vector{Vector3D}, roll::Float64, pitch::Float64, yaw::Float64)
    sinα, cosα = sincos(yaw)
    sinβ, cosβ = sincos(pitch)
    sinγ, cosγ = sincos(roll)

    m = [
        cosα*cosβ  cosα*sinβ*sinγ-sinα*cosγ  cosα*sinβ*cosγ+sinα*sinγ;
        sinα*cosβ  sinα*sinβ*sinγ+cosα*cosγ  sinα*sinβ*cosγ-cosα*sinγ;
        -sinβ      cosβ*sinγ                 cosβ*cosγ
    ]

    rotated_rays = Vector3D[]
    for ray in rays
        v = [ray.x, ray.y, ray.z]
        rotated = m * v
        push!(rotated_rays, Vector3D(rotated...))
    end

    return rotated_rays
end

function getRayGroundIntersections(rays::Vector{Vector3D}, origin::Vector3D)
    return [findRayGroundIntersection(ray, origin) for ray in rays]
end

function findRayGroundIntersection(ray::Vector3D, origin::Vector3D)
    t = -origin.z / ray.z
    Vector3D(
        origin.x + ray.x * t,
        origin.y + ray.y * t,
        origin.z + ray.z * t
    )
end

function sortClockwise(points::Vector{Vector3D})
    # Calculate the centroid
    centroid = Vector3D(
        sum(p.x for p in points) / length(points),
        sum(p.y for p in points) / length(points),
        sum(p.z for p in points) / length(points)
    )

    # Sort points based on their angle from the centroid
    sorted = sort(points, by = p -> atand(p.y - centroid.y, p.x - centroid.x))

    return sorted
end

function point_in_polygon(x, y, vertices)
    n = length(vertices)
    inside = false
    j = n
    for i in 1:n
        xi, yi = vertices[i].x, vertices[i].y
        xj, yj = vertices[j].x, vertices[j].y
        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)
            inside = !inside
        end
        j = i
    end
    return inside
end

function project_footprint_to_grid(bbox, grid_size_x, grid_size_y, cell_size)
    min_x, max_x = extrema(v.x for v in bbox)
    min_y, max_y = extrema(v.y for v in bbox)

    start_i = max(1, floor(Int, min_x / cell_size) + 1)
    end_i = min(grid_size_x, ceil(Int, max_x / cell_size))
    start_j = max(1, floor(Int, min_y / cell_size) + 1)
    end_j = min(grid_size_y, ceil(Int, max_y / cell_size))

    touched_cells = Vector{Int}[]

    for i in start_i:end_i
        for j in start_j:end_j
            x = (i - 0.5) * cell_size
            y = (j - 0.5) * cell_size
            if point_in_polygon(x, y, bbox)
                push!(touched_cells, [i, j])
            end
        end
    end

    return touched_cells
end