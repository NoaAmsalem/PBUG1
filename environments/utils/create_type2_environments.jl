using Luxor

const TAU = 2π
const MIN_FREE_WIDTH = 3
const DIR8 = [
    (dr, dc)
    for dr in -1:1 for dc in -1:1
    if !(dr == 0 && dc == 0)
]

# ============================================================
# Common occupancy-grid helpers
# ============================================================

@inline in_bounds(occ, r, c) =
    1 <= r <= size(occ, 1) && 1 <= c <= size(occ, 2)

function fill_rect!(occ::BitMatrix, r0::Int, c0::Int, r1::Int, c1::Int)
    nrows, ncols = size(occ)
    rr0 = clamp(min(r0, r1), 1, nrows)
    rr1 = clamp(max(r0, r1), 1, nrows)
    cc0 = clamp(min(c0, c1), 1, ncols)
    cc1 = clamp(max(c0, c1), 1, ncols)

    if rr0 <= rr1 && cc0 <= cc1
        occ[rr0:rr1, cc0:cc1] .= true
    end
    return nothing
end

function carve_rect!(occ::BitMatrix, r0::Int, c0::Int, r1::Int, c1::Int)
    nrows, ncols = size(occ)
    rr0 = clamp(min(r0, r1), 1, nrows)
    rr1 = clamp(max(r0, r1), 1, nrows)
    cc0 = clamp(min(c0, c1), 1, ncols)
    cc1 = clamp(max(c0, c1), 1, ncols)

    if rr0 <= rr1 && cc0 <= cc1
        occ[rr0:rr1, cc0:cc1] .= false
    end
    return nothing
end

function draw_cell(row::Int, col::Int, cellsize::Real, xoff::Real, yoff::Real)
    x = xoff + (col - 0.5) * cellsize
    y = yoff + (row - 0.5) * cellsize
    box(Point(x, y), cellsize, cellsize, :fill)
end

"""Render a Bool occupancy grid exactly as a black/white PNG."""
function render_occ_png(
    occ::BitMatrix;
    outpath::AbstractString,
)
    nrows, ncols = size(occ)
    mkpath(dirname(outpath))

    Drawing(ncols, nrows, outpath)
    origin()
    background("white")
    setcolor("black")

    xoff = -ncols / 2
    yoff = -nrows / 2

    for row in 1:nrows, col in 1:ncols
        occ[row, col] || continue
        draw_cell(row, col, 1, xoff, yoff)
    end

    finish()
    return outpath
end

"""
Write one start-goal pair in 1-based `(row, col)` coordinates.

Format:
    start_row start_col goal_row goal_col
"""
function write_pair_file(
    folder::AbstractString,
    index::Int,
    start::Tuple{Int,Int},
    goal::Tuple{Int,Int},
)
    pair_path = joinpath(folder, "pair_$(index).txt")
    open(pair_path, "w") do io
        println(io, "$(start[1]) $(start[2]) $(goal[1]) $(goal[2])")
    end
    return pair_path
end

@inline function angular_distance(a::Real, b::Real)
    return abs(mod(a - b + π, TAU) - π)
end

function require_min_width(name::AbstractString, width::Real)
    width >= MIN_FREE_WIDTH || throw(ArgumentError(
        "$name is only $(round(width; digits=2)) pixels wide; " *
        "environment requirements require at least $MIN_FREE_WIDTH pixels.",
    ))
    return nothing
end

# ============================================================
# Environment validation
# ============================================================

"""
Label occupied 8-connected components. Each component is treated as one obstacle.
"""
function obstacle_components(occ::BitMatrix)
    nrows, ncols = size(occ)
    labels = zeros(Int, nrows, ncols)
    component_count = 0

    queue = Tuple{Int,Int}[]

    for row in 1:nrows, col in 1:ncols
        if !occ[row, col] || labels[row, col] != 0
            continue
        end

        component_count += 1
        empty!(queue)
        push!(queue, (row, col))
        labels[row, col] = component_count
        head = 1

        while head <= length(queue)
            r, c = queue[head]
            head += 1

            for (dr, dc) in DIR8
                rr, cc = r + dr, c + dc
                if in_bounds(occ, rr, cc) &&
                   occ[rr, cc] &&
                   labels[rr, cc] == 0
                    labels[rr, cc] = component_count
                    push!(queue, (rr, cc))
                end
            end
        end
    end

    return labels, component_count
end

function free_path_exists(
    occ::BitMatrix,
    start::Tuple{Int,Int},
    goal::Tuple{Int,Int},
)
    nrows, ncols = size(occ)
    visited = falses(nrows, ncols)
    queue = Tuple{Int,Int}[start]
    visited[start...] = true
    head = 1

    while head <= length(queue)
        r, c = queue[head]
        head += 1

        (r, c) == goal && return true

        for (dr, dc) in DIR8
            rr, cc = r + dr, c + dc
            if in_bounds(occ, rr, cc) &&
               !occ[rr, cc] &&
               !visited[rr, cc]
                visited[rr, cc] = true
                push!(queue, (rr, cc))
            end
        end
    end

    return false
end

"""
Validate the generated map against the attached environment requirements.

Requirement 1 (no too-narrow loops/corridors inside one obstacle boundary) is
ensured by explicit >= 3-pixel geometric assertions in each generator below.
This validator checks the topology-dependent requirements:

2. Two distinct obstacles do not share a free boundary cell.
3. There is at least one non-boundary free pixel between the boundaries of
   distinct obstacles (8-neighbor interpretation).
4. No obstacle touches the image border, so every obstacle boundary is contained
   inside the environment.

It also checks that start and goal are free and connected in the true map.
"""
function validate_environment(
    occ::BitMatrix,
    start::Tuple{Int,Int},
    goal::Tuple{Int,Int};
    name::AbstractString="environment",
)
    nrows, ncols = size(occ)

    in_bounds(occ, start...) || error("$name: start $start is outside the map")
    in_bounds(occ, goal...) || error("$name: goal $goal is outside the map")
    !occ[start...] || error("$name: start $start is occupied")
    !occ[goal...] || error("$name: goal $goal is occupied")

    # Requirement 4: obstacle boundaries must be fully inside the image.
    if any(occ[1, :]) || any(occ[end, :]) ||
       any(occ[:, 1]) || any(occ[:, end])
        error("$name: an obstacle touches the image border")
    end

    labels, n_obstacles = obstacle_components(occ)

    # For each free cell, store which obstacle owns it as a boundary cell.
    boundary_owner = zeros(Int, nrows, ncols)

    for row in 1:nrows, col in 1:ncols
        occ[row, col] || continue
        obstacle_id = labels[row, col]

        for (dr, dc) in DIR8
            rr, cc = row + dr, col + dc
            if !in_bounds(occ, rr, cc) || occ[rr, cc]
                continue
            end

            owner = boundary_owner[rr, cc]
            if owner == 0
                boundary_owner[rr, cc] = obstacle_id
            elseif owner != obstacle_id
                error(
                    "$name: obstacles $owner and $obstacle_id share " *
                    "boundary cell ($rr, $cc)",
                )
            end
        end
    end

    # Requirement 3: distinct obstacle boundaries must not touch each other.
    # If two boundary cells are 8-neighbors, there is no free pixel between
    # the two boundaries.
    for row in 1:nrows, col in 1:ncols
        owner = boundary_owner[row, col]
        owner == 0 && continue

        for (dr, dc) in DIR8
            rr, cc = row + dr, col + dc
            in_bounds(occ, rr, cc) || continue
            other = boundary_owner[rr, cc]

            if other != 0 && other != owner
                error(
                    "$name: boundaries of obstacles $owner and $other " *
                    "have no free-pixel passage between them near " *
                    "($row, $col)",
                )
            end
        end
    end

    free_path_exists(occ, start, goal) || error(
        "$name: no 8-neighbor free path exists from $start to $goal",
    )

    return n_obstacles
end

function save_generated_environment!(
    folder::AbstractString,
    env_class::AbstractString,
    index::Int,
    generator::Function,
)
    mkpath(folder)
    map_path = joinpath(folder, "$(env_class)_$(index).png")
    _, occ, start, goal = generator(map_path)

    n_obstacles = validate_environment(
        occ,
        start,
        goal;
        name="$(env_class)_$(index)",
    )

    pair_path = write_pair_file(folder, index, start, goal)

    println(
        "saved $(env_class)_$(index): start=$start, goal=$goal, " *
        "obstacles=$n_obstacles, pair=$(basename(pair_path))",
    )

    return nothing
end

# ============================================================
# Rimon / CBUG family
# ============================================================

function make_rimon_cbug_png(;
    outpath="rimon.png",
    imgsize::Int=200,
    N::Int=4,
    fill_fraction::Float64=0.55,
    outer_radius_fraction::Float64=0.43,
    ring_gap_fraction::Float64=0.045,
    inner_start_radius_fraction::Float64=0.06,
    gap_center::Float64=0.0,
    gap_width::Float64=0.16,
    outline_width::Float64=2.0,
)
    N >= 2 || throw(ArgumentError("N must be at least 2"))
    0 < fill_fraction < 1 || throw(ArgumentError("fill_fraction must be in (0, 1)"))

    center = (imgsize ÷ 2, imgsize ÷ 2)
    outer_radius = outer_radius_fraction * imgsize
    ring_gap = ring_gap_fraction * imgsize
    inner_radius = outer_radius - ring_gap
    start_radius = inner_start_radius_fraction * imgsize

    delta = TAU / N
    wedge_width = fill_fraction * delta
    wedge_gap = delta - wedge_width

    require_min_width("Rimon wedge gap at its narrowest point", start_radius * wedge_gap)
    require_min_width("Rimon inner-ring opening", inner_radius * gap_width)
    require_min_width(
        "Rimon annular passage",
        outer_radius - inner_radius - outline_width,
    )

    occ = falses(imgsize, imgsize)

    for row in 1:imgsize, col in 1:imgsize
        y = row - center[1]
        x = col - center[2]
        radius = hypot(x, y)
        theta = atan(y, x)

        # Closed outer circular boundary.
        if abs(radius - outer_radius) <= outline_width / 2
            occ[row, col] = true
            continue
        end

        # Inner circular boundary, with a gap on the requested side.
        if abs(radius - inner_radius) <= outline_width / 2 &&
           angular_distance(theta, gap_center) > gap_width / 2
            occ[row, col] = true
            continue
        end

        # Radial wedges, connected to the inner circular boundary.
        if start_radius <= radius <= inner_radius
            phase = delta / 2
            for i in 0:(N - 1)
                wedge_center = phase + i * delta
                if angular_distance(theta, wedge_center) <= wedge_width / 2
                    occ[row, col] = true
                    break
                end
            end
        end
    end

    start = center
    goal = (center[1], 20)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Nested circular gates / loops family
# ============================================================

function make_nested_gates_png(;
    outpath="loops.png",
    imgsize::Int=200,
    outer_radius_fraction::Float64=0.46,
    nrings::Int=5,
    wall_fraction::Float64=0.025,
    gap_width::Float64=0.40,
    outer_outline_width::Float64=2.0,
)
    nrings >= 1 || throw(ArgumentError("nrings must be at least 1"))

    center = (imgsize ÷ 2, imgsize ÷ 2)
    outer_radius = outer_radius_fraction * imgsize
    wall = wall_fraction * imgsize
    radii = collect(range(0.14 * outer_radius, stop=0.86 * outer_radius, length=nrings))

    require_min_width("Nested-gates smallest opening", minimum(radii) * gap_width)

    if nrings > 1
        radial_gap = minimum(diff(radii)) - wall
        require_min_width("Nested-gates radial passage", radial_gap)
    end

    frame_radius = outer_radius + wall / 2
    require_min_width(
        "Nested-gates outer passage",
        frame_radius - radii[end] - wall / 2 - outer_outline_width / 2,
    )

    occ = falses(imgsize, imgsize)

    for row in 1:imgsize, col in 1:imgsize
        y = row - center[1]
        x = col - center[2]
        radius = hypot(x, y)
        theta = atan(y, x)

        if abs(radius - frame_radius) <= outer_outline_width / 2
            occ[row, col] = true
            continue
        end

        for (index, ring_radius) in enumerate(radii)
            gap_center = isodd(index) ? 0.0 : π
            if abs(radius - ring_radius) <= wall / 2 &&
               angular_distance(theta, gap_center) > gap_width / 2
                occ[row, col] = true
                break
            end
        end
    end

    start = center
    goal = (center[1], 15)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Double-comb / teeth corridor family
# ============================================================

function make_teeth_corridor_png(;
    outpath="dcomb.png",
    imgsize::Int=200,
    n_pairs::Int=2,
    tooth_inner_width::Int=3,
    tooth_height::Int=5,
    pair_gap::Int=4,
    left_tail::Int=2,
    right_tail::Int=10,
    top_margin::Int=2,
    bottom_margin::Int=2,
    passage_height::Int=3,
    block_left::Bool=true,
    outer_margin::Int=4,
)
    n_pairs >= 1 || throw(ArgumentError("n_pairs must be at least 1"))
    require_min_width("D-comb tooth opening", tooth_inner_width)
    require_min_width("D-comb middle passage", passage_height)
    require_min_width("D-comb gap between tooth pairs", pair_gap)

    tooth_width = tooth_inner_width + 2
    logical_cols = left_tail + n_pairs * tooth_width + (n_pairs - 1) * pair_gap + right_tail
    logical_rows = top_margin + tooth_height + 1 + passage_height + 1 + tooth_height + bottom_margin

    logical = falses(logical_rows, logical_cols)
    upper_row = top_margin + tooth_height + 1
    lower_row = upper_row + passage_height + 1

    logical[upper_row, :] .= true
    logical[lower_row, :] .= true

    if block_left
        logical[upper_row:lower_row, 1] .= true
    end

    pair_starts = [
        left_tail + 1 + (k - 1) * (tooth_width + pair_gap)
        for k in 1:n_pairs
    ]

    for left_col in pair_starts
        right_col = left_col + tooth_width - 1
        inner_left = left_col + 1
        inner_right = right_col - 1

        logical[upper_row, inner_left:inner_right] .= false
        logical[lower_row, inner_left:inner_right] .= false

        top_row = upper_row - tooth_height
        logical[top_row:upper_row, left_col] .= true
        logical[top_row:upper_row, right_col] .= true
        logical[top_row, left_col:right_col] .= true

        bottom_row = lower_row + tooth_height
        logical[lower_row:bottom_row, left_col] .= true
        logical[lower_row:bottom_row, right_col] .= true
        logical[bottom_row, left_col:right_col] .= true
    end

    scale = floor(Int, min(
        (imgsize - 2 * outer_margin) / logical_cols,
        (imgsize - 2 * outer_margin) / logical_rows,
    ))
    scale >= 1 || error("D-comb logical map does not fit inside $imgsize x $imgsize")

    occ = falses(imgsize, imgsize)
    draw_height = logical_rows * scale
    draw_width = logical_cols * scale
    row_offset = fld(imgsize - draw_height, 2)
    col_offset = fld(imgsize - draw_width, 2)

    for row in 1:logical_rows, col in 1:logical_cols
        logical[row, col] || continue

        row0 = row_offset + (row - 1) * scale + 1
        row1 = row_offset + row * scale
        col0 = col_offset + (col - 1) * scale + 1
        col1 = col_offset + col * scale
        fill_rect!(occ, row0, col0, row1, col1)
    end

    start = (imgsize ÷ 2, imgsize ÷ 2)
    goal = (imgsize ÷ 2, 5)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Nested courtyard family
# ============================================================

function make_nested_courtyard_png(;
    outpath="courtyard.png",
    imgsize::Int=200,
    n_courtyards::Int=4,
    wall_thickness::Int=4,
    inner_half_size::Int=14,
    outer_half_size::Int=58,
    opening_depth::Int=16,
    goal_side::Symbol=:left,
    first_opening_away::Bool=true,
    goal_offset::Int=7,
)
    n_courtyards >= 1 || throw(ArgumentError("n_courtyards must be at least 1"))
    iseven(imgsize) || throw(ArgumentError("imgsize must be even"))
    goal_side in (:left, :right) || throw(ArgumentError("goal_side must be :left or :right"))
    wall_thickness >= 1 || throw(ArgumentError("wall_thickness must be positive"))
    opening_depth > wall_thickness || throw(ArgumentError("opening_depth must exceed wall_thickness"))

    center_row = imgsize ÷ 2 + 1
    center_col = imgsize ÷ 2 + 1

    half_sizes = if n_courtyards == 1
        [outer_half_size]
    else
        round.(Int, collect(range(
            inner_half_size,
            stop=outer_half_size,
            length=n_courtyards,
        )))
    end

    length(unique(half_sizes)) == n_courtyards || throw(ArgumentError(
        "Too many courtyards for the selected size interval",
    ))

    if n_courtyards > 1
        min_spacing = minimum(diff(half_sizes))
        required_spacing = wall_thickness + MIN_FREE_WIDTH
        min_spacing >= required_spacing || throw(ArgumentError(
            "Courtyard walls leave less than $MIN_FREE_WIDTH free pixels " *
            "between obstacle boundaries. Increase the size interval or " *
            "reduce n_courtyards/wall_thickness.",
        ))
    end

    opening_depth < 2 * minimum(half_sizes) + 1 - wall_thickness ||
        throw(ArgumentError("opening_depth is too large for the innermost courtyard"))

    outer_half_size + goal_offset < imgsize ÷ 2 || throw(ArgumentError(
        "outer_half_size and goal_offset leave no room for the goal",
    ))

    occ = falses(imgsize, imgsize)
    opposite(side::Symbol) = side == :left ? :right : :left
    innermost_opening = first_opening_away ? opposite(goal_side) : goal_side

    for (index, half_size) in enumerate(half_sizes)
        opening_side = isodd(index) ? innermost_opening : opposite(innermost_opening)

        top = center_row - half_size
        bottom = center_row + half_size
        left = center_col - half_size
        right = center_col + half_size
        t = wall_thickness
        d = opening_depth

        if opening_side == :left
            fill_rect!(occ, top, left + d, top + t - 1, right)
            fill_rect!(occ, bottom - t + 1, left + d, bottom, right)
            fill_rect!(occ, top, right - t + 1, bottom, right)
        else
            fill_rect!(occ, top, left, top + t - 1, right - d)
            fill_rect!(occ, bottom - t + 1, left, bottom, right - d)
            fill_rect!(occ, top, left, bottom, left + t - 1)
        end
    end

    start = (center_row, center_col)
    goal = goal_side == :left ?
        (center_row, center_col - outer_half_size - goal_offset) :
        (center_row, center_col + outer_half_size + goal_offset)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Bus-terminal family
# ============================================================

function make_bus_terminal_platform_png(;
    outpath="busterminal.png",
    imgsize::Int=200,
    n_spikes::Int=0,
    outer_margin::Int=4,
    obstacle_margin::Int=10,
    corridor_width::Int=10,
    top_stub_height::Int=16,
    top_bar_half_length::Int=48,
    bottom_bar_half_length::Int=48,
    side_corridor_offset::Int=44,
    central_room_height::Int=78,
    central_room_width::Int=48,
    central_room_top::Int=58,
    connector_width::Int=48,
    spike_length::Int=10,
    spike_height::Int=8,
    spike_end_margin::Int=18,
    start_stub_offset::Int=5,
    goal_offset_from_room_top::Int=16,
)
    imgsize == 200 || throw(ArgumentError("bus-terminal geometry currently assumes imgsize=200"))
    n_spikes >= 0 || throw(ArgumentError("n_spikes must be nonnegative"))
    obstacle_margin > outer_margin >= 2 || throw(ArgumentError("invalid outer/obstacle margins"))

    require_min_width("Bus-terminal corridor", corridor_width)
    require_min_width("Bus-terminal connector", connector_width)
    require_min_width("Bus-terminal spike height", spike_height)

    center_col = 101
    half_width = fld(corridor_width, 2)
    occ = falses(imgsize, imgsize)

    fill_rect!(
        occ,
        obstacle_margin + 1,
        obstacle_margin + 1,
        imgsize - obstacle_margin,
        imgsize - obstacle_margin,
    )

    top_bar_r0 = 36
    top_bar_r1 = top_bar_r0 + corridor_width - 1
    bottom_bar_r0 = 156
    bottom_bar_r1 = bottom_bar_r0 + corridor_width - 1

    left_c0 = center_col - side_corridor_offset - half_width
    left_c1 = left_c0 + corridor_width - 1
    right_c0 = center_col + side_corridor_offset - half_width
    right_c1 = right_c0 + corridor_width - 1

    top_c0 = center_col - top_bar_half_length
    top_c1 = center_col + top_bar_half_length
    bottom_c0 = center_col - bottom_bar_half_length
    bottom_c1 = center_col + bottom_bar_half_length

    room_r0 = central_room_top
    room_r1 = room_r0 + central_room_height - 1
    room_c0 = center_col - fld(central_room_width, 2)
    room_c1 = room_c0 + central_room_width - 1

    connector_c0 = center_col - fld(connector_width, 2)
    connector_c1 = connector_c0 + connector_width - 1

    room_r1 < bottom_bar_r0 - 2 || throw(ArgumentError(
        "Central room must leave a free connector before the bottom bar",
    ))

    left_available = room_c0 - left_c1 - 1
    right_available = right_c0 - room_c1 - 1
    max_spike_length = min(left_available, right_available) - MIN_FREE_WIDTH
    spike_length <= max_spike_length || throw(ArgumentError(
        "spike_length=$spike_length is too large; maximum is $max_spike_length",
    ))

    carve_rect!(occ, top_bar_r0, top_c0, top_bar_r1, top_c1)
    carve_rect!(
        occ,
        top_bar_r0 - top_stub_height,
        center_col - half_width,
        top_bar_r0,
        center_col + half_width,
    )
    carve_rect!(occ, top_bar_r0, left_c0, bottom_bar_r1, left_c1)
    carve_rect!(occ, top_bar_r0, right_c0, bottom_bar_r1, right_c1)
    carve_rect!(occ, bottom_bar_r0, bottom_c0, bottom_bar_r1, bottom_c1)
    carve_rect!(occ, room_r0, room_c0, room_r1, room_c1)
    carve_rect!(occ, room_r1 + 1, connector_c0, bottom_bar_r1, connector_c1)

    if n_spikes > 0
        spike_zone_r0 = top_bar_r1 + spike_end_margin
        spike_zone_r1 = bottom_bar_r0 - spike_end_margin
        available_span = spike_zone_r1 - spike_zone_r0
        required_spacing = spike_height + MIN_FREE_WIDTH
        max_spikes = n_spikes == 1 ? 1 : fld(available_span, required_spacing) + 1
        n_spikes <= max_spikes || throw(ArgumentError(
            "Requested $n_spikes spikes, but at most $max_spikes fit safely",
        ))

        centers = n_spikes == 1 ?
            [fld(spike_zone_r0 + spike_zone_r1, 2)] :
            round.(Int, collect(range(spike_zone_r0, stop=spike_zone_r1, length=n_spikes)))

        for spike_row in centers
            r0 = spike_row - fld(spike_height, 2)
            r1 = r0 + spike_height - 1
            carve_rect!(occ, r0, left_c1, r1, left_c1 + spike_length)
            carve_rect!(occ, r0, right_c0 - spike_length, r1, right_c0)
        end
    end

    start = (top_bar_r0 - top_stub_height + start_stub_offset, center_col)
    goal = (room_r0 + goal_offset_from_room_top, center_col)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Alternating connected rows family
# ============================================================

function make_alternating_connected_rows_png(;
    outpath="parallelrows.png",
    imgsize::Int=200,
    n_rows::Int=2,
    outer_margin::Int=5,
    block_side_margin::Int=12,
    side_corridor_width::Int=14,
    horizontal_corridor_height::Int=11,
    row_height::Int=17,
    start_stub_height::Int=14,
    start_stub_width::Int=14,
    top_bottom_padding::Int=7,
    internal_gap_width::Int=18,
)
    imgsize == 200 || throw(ArgumentError("parallel-row geometry currently assumes imgsize=200"))
    n_rows >= 1 || throw(ArgumentError("n_rows must be at least 1"))
    block_side_margin > outer_margin >= 2 || throw(ArgumentError("invalid margins"))

    require_min_width("Parallel-row side corridor", side_corridor_width)
    require_min_width("Parallel-row horizontal corridor", horizontal_corridor_height)
    require_min_width("Parallel-row start stub", start_stub_width)

    network_height =
        2 * horizontal_corridor_height +
        n_rows * row_height +
        (n_rows - 1) * horizontal_corridor_height

    block_height = network_height + start_stub_height + 2 * top_bottom_padding
    block_height <= imgsize - 2 * outer_margin || throw(ArgumentError(
        "n_rows=$n_rows does not fit with the current dimensions",
    ))

    occ = falses(imgsize, imgsize)
    center_row = fld(imgsize + 1, 2)
    center_col = fld(imgsize + 1, 2)

    block_r0 = center_row - fld(block_height, 2)
    block_r1 = block_r0 + block_height - 1
    block_c0 = block_side_margin + 1
    block_c1 = imgsize - block_side_margin
    fill_rect!(occ, block_r0, block_c0, block_r1, block_c1)

    left_c0 = block_c0 + 8
    left_c1 = left_c0 + side_corridor_width - 1
    right_c1 = block_c1 - 8
    right_c0 = right_c1 - side_corridor_width + 1

    top_r0 = block_r0 + top_bottom_padding + start_stub_height
    top_r1 = top_r0 + horizontal_corridor_height - 1
    bottom_r0 =
        top_r0 +
        n_rows * row_height +
        (n_rows - 1) * horizontal_corridor_height
    bottom_r1 = bottom_r0 + horizontal_corridor_height - 1

    carve_rect!(occ, top_r0, left_c0, bottom_r1, left_c1)
    carve_rect!(occ, top_r0, right_c0, bottom_r1, right_c1)
    carve_rect!(occ, top_r0, left_c0, top_r1, right_c1)
    carve_rect!(occ, bottom_r0, left_c0, bottom_r1, right_c1)

    stub_c0 = center_col - fld(start_stub_width, 2)
    stub_c1 = stub_c0 + start_stub_width - 1
    stub_r0 = top_r0 - start_stub_height
    stub_r1 = top_r0
    carve_rect!(occ, stub_r0, stub_c0, stub_r1, stub_c1)

    for i in 1:(n_rows - 1)
        passage_r0 =
            top_r1 + 1 +
            i * row_height +
            (i - 1) * horizontal_corridor_height
        passage_r1 = passage_r0 + horizontal_corridor_height - 1

        if isodd(i)
            carve_rect!(
                occ,
                passage_r0,
                left_c0,
                passage_r1,
                right_c0 - internal_gap_width,
            )
        else
            carve_rect!(
                occ,
                passage_r0,
                left_c1 + internal_gap_width,
                passage_r1,
                right_c1,
            )
        end
    end

    start = (stub_r0 + fld(start_stub_height, 2), center_col)
    goal = (bottom_r0 + fld(horizontal_corridor_height, 2), center_col)

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Nested split-room family
# ============================================================

function carve_frame!(
    occ::BitMatrix,
    r0::Int,
    c0::Int,
    r1::Int,
    c1::Int;
    corridor_width::Int,
)
    w = corridor_width
    carve_rect!(occ, r0, c0, r0 + w - 1, c1)
    carve_rect!(occ, r1 - w + 1, c0, r1, c1)
    carve_rect!(occ, r0, c0, r1, c0 + w - 1)
    carve_rect!(occ, r0, c1 - w + 1, r1, c1)
    return nothing
end

function make_nested_splitroom_png(;
    outpath="nestedsplitroom.png",
    imgsize::Int=200,
    n_layers::Int=1,
    room_height::Int=84,
    room_width::Int=56,
    room_shift_right::Int=10,
    corridor_width::Int=6,
    gap::Int=6,
    start_offset_from_room::Int=10,
    block_margin::Int=5,
)
    imgsize == 200 || throw(ArgumentError("split-room geometry currently assumes imgsize=200"))
    n_layers >= 1 || throw(ArgumentError("n_layers must be at least 1"))
    block_margin >= 2 || throw(ArgumentError("block_margin must be at least 2"))

    require_min_width("Split-room corridor", corridor_width)
    require_min_width("Split-room gap", gap)

    # Important validity fix: the original version started with an all-black
    # image, so the obstacle touched the image border. We now use an inset
    # black block, preserving the internal topology while satisfying requirement 4.
    occ = falses(imgsize, imgsize)
    fill_rect!(
        occ,
        block_margin + 1,
        block_margin + 1,
        imgsize - block_margin,
        imgsize - block_margin,
    )

    center_row = fld(imgsize + 1, 2)
    center_col = fld(imgsize + 1, 2) + room_shift_right
    half_width = fld(corridor_width, 2)
    step = gap + corridor_width

    room_r0 = center_row - fld(room_height, 2)
    room_r1 = room_r0 + room_height - 1
    room_c0 = center_col - fld(room_width, 2)
    room_c1 = room_c0 + room_width - 1
    carve_rect!(occ, room_r0, room_c0, room_r1, room_c1)

    frame_bounds = Tuple{Int,Int,Int,Int}[]

    for layer in 1:n_layers
        r0 = room_r0 - layer * step
        r1 = room_r1 + layer * step
        c0 = room_c0 - layer * step
        c1 = room_c1 + layer * step

        r0 > block_margin + 1 &&
        c0 > block_margin + 1 &&
        r1 < imgsize - block_margin &&
        c1 < imgsize - block_margin || throw(ArgumentError(
            "n_layers=$n_layers does not fit inside the inset obstacle block",
        ))

        carve_frame!(
            occ,
            r0,
            c0,
            r1,
            c1;
            corridor_width=corridor_width,
        )
        push!(frame_bounds, (r0, c0, r1, c1))
    end

    _, outer_c0, _, outer_c1 = frame_bounds[end]
    mid_r0 = center_row - half_width
    mid_r1 = center_row + half_width

    # White passage from the room to the leftmost vertical passage.
    carve_rect!(occ, mid_r0, outer_c0, mid_r1, room_c0)

    # Black separator from the room to just before the rightmost vertical
    # passage, leaving that vertical passage white.
    rightmost_vertical_c0 = outer_c1 - corridor_width + 1
    separator_c0 = room_c1 + 1
    separator_c1 = rightmost_vertical_c0 - 1
    separator_c0 <= separator_c1 || error("Split-room separator has invalid geometry")
    fill_rect!(occ, mid_r0, separator_c0, mid_r1, separator_c1)

    start = (center_row, room_c0 - start_offset_from_room)
    goal = (center_row, outer_c1 - fld(corridor_width, 2))

    render_occ_png(occ; outpath=outpath)
    return outpath, occ, start, goal
end

# ============================================================
# Dataset generation
# ============================================================

function main()
    environments_path = dirname(@__DIR__)
    type2_path = joinpath(environments_path, "type2")

    rm(type2_path; force=true, recursive=true)
    mkpath(type2_path)

    rimon_path = joinpath(type2_path, "rimon")
    for n in 2:2:8
        index = n ÷ 2
        save_generated_environment!(
            rimon_path,
            "rimon",
            index,
            outpath -> make_rimon_cbug_png(
                outpath=outpath,
                imgsize=200,
                N=n,
            ),
        )
    end

    loops_path = joinpath(type2_path, "loops")
    for n in 2:5
        index = n - 1
        save_generated_environment!(
            loops_path,
            "loops",
            index,
            outpath -> make_nested_gates_png(
                outpath=outpath,
                imgsize=200,
                nrings=n,
            ),
        )
    end

    dcomb_path = joinpath(type2_path, "dcomb")
    for n in 1:4
        save_generated_environment!(
            dcomb_path,
            "dcomb",
            n,
            outpath -> make_teeth_corridor_png(
                outpath=outpath,
                imgsize=200,
                n_pairs=n,
            ),
        )
    end

    courtyard_path = joinpath(type2_path, "courtyard")
    for n in 2:7
        index = n - 1
        save_generated_environment!(
            courtyard_path,
            "courtyard",
            index,
            outpath -> make_nested_courtyard_png(
                outpath=outpath,
                imgsize=200,
                n_courtyards=n,
                first_opening_away=true,
            ),
        )
    end

    busterminal_path = joinpath(type2_path, "busterminal")
    for n in 0:2
        index = n + 1
        save_generated_environment!(
            busterminal_path,
            "busterminal",
            index,
            outpath -> make_bus_terminal_platform_png(
                outpath=outpath,
                n_spikes=n,
            ),
        )
    end

    parallelrows_path = joinpath(type2_path, "parallelrows")
    for n in 1:5
        save_generated_environment!(
            parallelrows_path,
            "parallelrows",
            n,
            outpath -> make_alternating_connected_rows_png(
                outpath=outpath,
                n_rows=n,
            ),
        )
    end

    splitroom_path = joinpath(type2_path, "nestedsplitroom")
    for n in 1:3
        save_generated_environment!(
            splitroom_path,
            "nestedsplitroom",
            n,
            outpath -> make_nested_splitroom_png(
                outpath=outpath,
                n_layers=n,
            ),
        )
    end

    println("Saved and validated deceptive environments to: $type2_path")
    println("Each map has a matching pair_<index>.txt in 1-based row/col format.")
    return nothing
end

main()
