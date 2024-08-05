function visualize_camera_polygon(camera_pos, touched_cells, bbox, grid_size_x = 50, grid_size_y = 50, cell_size = 10.0)
    camera_x, camera_y, camera_z = camera_pos

    x = [p.x for p in bbox]
    y = [p.y for p in bbox]
    z = [p.z for p in bbox]

    fig = Figure(size = (800, 600))
    ax = Axis3(fig[1, 1], 
               aspect = :data,
               xlabel = "X", ylabel = "Y", zlabel = "Z",
               title = "Camera Bounding Polygon with Grid")

    # Plot the ground polygon
    poly!(ax, [Point2f0(x[i], y[i]) for i in 1:4], color = (:lightblue, 0.5), transparency = true, shading=NoShading)
    
    # Plot the camera position
    scatter!(ax, Point3f0(camera_x, camera_y, camera_z), color = :red, markersize = 10)

    # Plot lines from camera to polygon corners
    for i in 1:4
        lines!(ax, [Point3f0(camera_x, camera_y, camera_z), Point3f0(x[i], y[i], z[i])], color = :gray, linewidth = 0.5)
    end

    # Plot the grid
    grid_colors = [([i, j] in touched_cells) ? RGBA(0.0, 1.0, 0.0, 0.3) : RGBA(1.0, 0.0, 0.0, 0.1) for i in 1:grid_size_x, j in 1:grid_size_y]
    heatmap!(ax, 0:cell_size:((grid_size_x)*cell_size), 0:cell_size:((grid_size_y)*cell_size), grid_colors)

    # Add grid lines
    for i in 0:grid_size_x
        lines!(ax, [Point3f0(i*cell_size, 0, 0), Point3f0(i*cell_size, grid_size_y*cell_size, 0)], 
               color = :black, linewidth = 0.5)
    end
    for j in 0:grid_size_y
        lines!(ax, [Point3f0(0, j*cell_size, 0), Point3f0(grid_size_x*cell_size, j*cell_size, 0)], 
               color = :black, linewidth = 0.5)
    end

    update_view!(ax, x, y, z)
    display(fig)
end

function update_view!(ax, x, y, z)
    xmin, xmax = extrema(x)
    ymin, ymax = extrema(y)
    zmin, zmax = extrema([z; 117.1])

    padding = 0.1 * max(xmax-xmin, ymax-ymin, zmax-zmin)
    
    limits!(ax, 
        xmin - padding, xmax + padding,
        ymin - padding, ymax + padding,
        zmin, zmax + padding
    )

    ax.azimuth[] = 0.7
    ax.elevation[] = 0.3
end