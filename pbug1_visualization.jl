module PBUG1Visualization

using Plots
using ..PBUG1

export plot_pbug1, save_pbug1_plot

coordinates(path) = ([p[2] for p in path], [p[1] for p in path])

"""Executed 4-connected path length, excluding repeated stationary samples."""
function executed_path_length(path)
    length(path) < 2 && return 0
    return sum(abs(path[i][1] - path[i - 1][1]) +
               abs(path[i][2] - path[i - 1][2])
               for i in 2:length(path))
end

"""
    plot_pbug1(world, result; title="PBUG1 execution")

Plot an occupancy grid and the complete trajectories stored in a
`PBUG1Result`. Obstacles are black, the left robot is blue, and the right robot
is orange. Hit and leave points are shown only when they exist.

The returned value is a regular `Plots.Plot`, so it can be displayed in a
notebook or customized with the normal Plots.jl API.
"""
function plot_pbug1(world::GridWorld, result::PBUG1Result;
                    title::AbstractString="PBUG1 execution",
                    cell_grid::Bool=true,
                    show_summary::Bool=true)
    rows, cols = size(world.occupied)

    plot_title = title
    if show_summary
        status = uppercasefirst(replace(String(result.status), '_' => ' '))
        left_length = executed_path_length(result.left_path)
        right_length = executed_path_length(result.right_path)
        summary = "Status: $status  |  Left path: $left_length  |  " *
                  "Right path: $right_length  |  Hits: $(length(result.hit_points))  |  " *
                  "Iterations: $(result.iterations)"
        plot_title = "$title\n$summary"
    end

    # Plots indexes image data as z[y, x], which matches grid[row, column].
    plt = heatmap(
        1:cols,
        1:rows,
        Float64.(world.occupied);
        color=cgrad([:white, :black], categorical=true),
        colorbar=false,
        clims=(0, 1),
        aspect_ratio=:equal,
        yflip=true,
        xlims=(0.5, cols + 0.5),
        # `yflip=true` performs the reversal. Supplying reversed limits as well
        # makes GR apply the transformation twice and collapse the row axis.
        ylims=(0.5, rows + 0.5),
        xlabel="column",
        ylabel="row",
        title=plot_title,
        legend=:outerright,
        grid=false,
        framestyle=:box,
        size=(800, 650),
    )

    # Cell centers have integer coordinates; therefore cell boundaries lie at
    # half-integers. Draw these before the paths so the trajectories stay clear.
    if cell_grid
        vline!(plt, collect(0.5:1.0:(cols + 0.5));
               color=:gray50, linewidth=0.5, alpha=0.65, label=false)
        hline!(plt, collect(0.5:1.0:(rows + 0.5));
               color=:gray50, linewidth=0.5, alpha=0.65, label=false)
    end

    lx, ly = coordinates(result.left_path)
    rx, ry = coordinates(result.right_path)

    plot!(plt, lx, ly;
          color=:dodgerblue,
          linewidth=3,
          label="left robot",
          marker=:circle,
          markersize=2,
          markerstrokewidth=0)

    plot!(plt, rx, ry;
          color=:darkorange,
          linewidth=2.5,
          linestyle=:dash,
          label="right robot",
          marker=:diamond,
          markersize=2,
          markerstrokewidth=0)

    start = first(result.left_path)
    scatter!(plt, [start[2]], [start[1]];
             marker=:star5, markersize=10, color=:limegreen,
             markerstrokecolor=:black, label="start")

    # When reached, the last point is the target. Otherwise the target is not
    # stored in PBUG1Result, so callers may pass it explicitly using the method
    # below.
    if result.status == :reached
        target = last(result.left_path)
        scatter!(plt, [target[2]], [target[1]];
                 marker=:star5, markersize=10, color=:red,
                 markerstrokecolor=:black, label="target")
    end

    if !isempty(result.hit_points)
        hx, hy = coordinates(result.hit_points)
        scatter!(plt, hx, hy;
                 marker=:diamond, markersize=6, color=:purple,
                 markerstrokecolor=:black, label="hit points")
    end

    # L0 is the start rather than a leave point created after an obstacle.
    if length(result.leave_points) > 1
        qx, qy = coordinates(result.leave_points[2:end])
        scatter!(plt, qx, qy;
                 marker=:rect, markersize=6, color=:cyan,
                 markerstrokecolor=:black, label="leave points")
    end

    return plt
end

"""Plot overload that always displays a separately supplied target cell."""
function plot_pbug1(world::GridWorld, result::PBUG1Result, target::Cell;
                    title::AbstractString="PBUG1 execution",
                    cell_grid::Bool=true,
                    show_summary::Bool=true)
    plt = plot_pbug1(world, result;
                     title=title,
                     cell_grid=cell_grid,
                     show_summary=show_summary)
    if result.status != :reached
        scatter!(plt, [target[2]], [target[1]];
                 marker=:star5, markersize=10, color=:red,
                 markerstrokecolor=:black, label="target")
    end
    return plt
end

"""Create the visualization and save it as PNG, PDF, SVG, or another Plots format."""
function save_pbug1_plot(filename::AbstractString, world::GridWorld,
                         result::PBUG1Result, target::Cell; kwargs...)
    plt = plot_pbug1(world, result, target; kwargs...)
    savefig(plt, filename)
    return filename
end

end # module
