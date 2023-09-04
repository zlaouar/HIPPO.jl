function sortclosest(pointlist, currpos)
    dists = [HIPPO.dist(pos, currpos) for pos ∈ pointlist]
    perm = sortperm(dists)
    newpoints = pointlist[perm]
    return newpoints
end

function gendists(pospoints, robotinit)
    finalpoints = Vector{Int64}[]
    pointlist = deepcopy(pospoints)
    currpos = robotinit
    for i in 1:length(pospoints)-1
        newpoints = sortclosest(pointlist, currpos)
        pointlist = newpoints[2:end]
        push!(finalpoints, newpoints[1])
        currpos = newpoints[1]
    end

    return finalpoints
end

function getdata(inputs, db, mapsize)
    pospoints = [db.ID2grid[inputs.user_points_p[i]] for i in 1:length(inputs.user_points_p)]
    #polys = [sketch.centroid for sketch in inputs.sketch_set]
    #polys = collect.([Int.(round.(pol)) for pol in polys])
    #pospoints = [pospoints; polys]
    return [ind2pos(mapsize, ind) for ind ∈ pospoints]
end

function polypoints(inputs, db, mapsize)
    sketchIDs = [sketch.ID_points for sketch ∈ inputs.sketch_set]
    sketchIDs = vcat(sketchIDs...)
    polypoints = [db.ID2grid[ID] for ID ∈ sketchIDs]
    return [ind2pos(mapsize, ind) for ind ∈ polypoints]
end

function gridpoints(mapsize)
    points = Vector{Int64}[]
    debut = 1
    fin = mapsize[2]
    for i in 1:mapsize[1]
        if mod(i,2) == 0 
            debut = mapsize[2]
            fin = 1
            for j ∈ debut:-1:fin
                push!(points, [i,j])
            end
        else
            debut = 1
            fin = mapsize[2]
            for j ∈ debut:fin
                push!(points, [i,j])
            end
        end
        
    end
    return points[1:2]
end