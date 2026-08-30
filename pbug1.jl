module PBUG1

export Cell, GridWorld, PBUG1Result, pbug1

const Cell = CartesianIndex{2}
const DIRS = (Cell(-1, 0), Cell(0, 1), Cell(1, 0), Cell(0, -1))

"""A true occupancy grid. `true` denotes an obstacle."""
struct GridWorld
    occupied::BitMatrix
end

struct PBUG1Result
    status::Symbol
    left_path::Vector{Cell}
    right_path::Vector{Cell}
    hit_points::Vector{Cell}
    leave_points::Vector{Cell}
    iterations::Int
end

inside(w::GridWorld, p::Cell) =
    1 <= p[1] <= size(w.occupied, 1) && 1 <= p[2] <= size(w.occupied, 2)

blocked(w::GridWorld, p::Cell) = !inside(w, p) || w.occupied[p]

sqdist(a::Cell, b::Cell) = (a[1] - b[1])^2 + (a[2] - b[2])^2

dir_index(d::Cell) = findfirst(==(d), DIRS)

"""One 4-connected digital-line step toward `goal`."""
function line_step(p::Cell, goal::Cell)
    dr, dc = goal[1] - p[1], goal[2] - p[2]
    dr == 0 && dc == 0 && return Cell(0, 0)
    if abs(dr) >= abs(dc)
        return Cell(sign(dr), 0)
    else
        return Cell(0, sign(dc))
    end
end

function append_move!(path::Vector{Cell}, p::Cell)
    path[end] == p || push!(path, p)
end

"""
Take one wall-following step. `hand=:left` keeps the wall on the left;
`hand=:right` keeps it on the right. The returned contact set contains every
blocked cell touched by the local collision sensor during this decision.
"""
function wall_step(w::GridWorld, p::Cell, heading::Cell, hand::Symbol)
    i = dir_index(heading)
    i === nothing && error("invalid heading $heading")
    offsets = hand === :left ? (-1, 0, 1, 2) : (1, 0, -1, 2)
    contacts = Cell[]
    for off in offsets
        j = mod1(i + off, 4)
        q = p + DIRS[j]
        if blocked(w, q)
            push!(contacts, q)
        else
            return q, DIRS[j], contacts
        end
    end
    return p, heading, contacts
end

function initial_heading(w::GridWorld, hit::Cell, incoming::Cell, hand::Symbol)
    i = dir_index(incoming)
    turns = hand === :left ? (-1, -2, 1) : (1, 2, -1)
    for off in turns
        d = DIRS[mod1(i + off, 4)]
        !blocked(w, hit + d) && return d
    end
    return -incoming
end

"""Append a shared path segment to both robot histories."""
function append_shared!(left::Vector{Cell}, right::Vector{Cell}, segment)
    for p in segment
        append_move!(left, p)
        append_move!(right, p)
    end
end

"""
    pbug1(world, start, target; max_steps=1_000_000)

Discrete occupancy-grid implementation of PBUG1. Both robots begin at `start`.
They use only collision queries made by straight-line and wall-following motion.

Statuses are `:reached`, `:unreachable`, `:trapped`, or `:step_limit`.
The two returned paths are complete executed trajectories, including the shared
motion before a hit and after the robots meet.
"""
function pbug1(world::GridWorld, start::Cell, target::Cell;
               max_steps::Int=1_000_000)
    blocked(world, start) && throw(ArgumentError("start is blocked"))
    blocked(world, target) && return PBUG1Result(
        :unreachable, [start], [start], Cell[], [start], 0)

    left = Cell[start]
    right = Cell[start]
    hits = Cell[]
    leaves = Cell[start]
    current = start
    steps = 0
    iteration = 0

    while steps < max_steps
        iteration += 1

        # Shared motion on the current M-line toward the target.
        incoming = Cell(0, 0)
        while current != target && steps < max_steps
            incoming = line_step(current, target)
            next = current + incoming
            blocked(world, next) && break
            current = next
            append_shared!(left, right, (current,))
            steps += 1
        end
        current == target && return PBUG1Result(
            :reached, left, right, hits, leaves, iteration)
        steps >= max_steps && break

        hit = current
        obstacle_seed = hit + incoming
        push!(hits, hit)

        # Split and follow the obstacle in opposite directions.
        lp = rp = hit
        lh = initial_heading(world, hit, incoming, :left)
        rh = initial_heading(world, hit, incoming, :right)
        lbranch = Cell[hit]
        rbranch = Cell[hit]
        lbest = rbest = hit
        observed_obstacle = Set{Cell}((obstacle_seed,))
        seen_joint_states = Set{Tuple{Cell,Cell,Cell,Cell}}()
        met = false

        while steps < max_steps
            state = (lp, lh, rp, rh)
            if state in seen_joint_states
                return PBUG1Result(:unreachable, left, right, hits, leaves, iteration)
            end
            push!(seen_joint_states, state)

            oldlp, oldrp = lp, rp
            # RL initially turns left and therefore keeps the obstacle on its
            # right; RR initially turns right and keeps it on its left.
            lp, lh, lc = wall_step(world, lp, lh, :right)
            rp, rh, rc = wall_step(world, rp, rh, :left)
            union!(observed_obstacle, lc)
            union!(observed_obstacle, rc)
            push!(lbranch, lp)
            push!(rbranch, rp)
            append_move!(left, lp)
            append_move!(right, rp)
            sqdist(lp, target) < sqdist(lbest, target) && (lbest = lp)
            sqdist(rp, target) < sqdist(rbest, target) && (rbest = rp)
            steps += 2

            if lp == target || rp == target
                return PBUG1Result(:reached, left, right, hits, leaves, iteration)
            end

            # Same vertex or simultaneous traversal of the same edge.
            if lp == rp || (oldlp == rp && oldrp == lp)
                met = true
                break
            end
        end
        met || break

        # Select Qm and reunite by reversing the winning robot's branch.
        use_left = sqdist(lbest, target) <= sqdist(rbest, target)
        branch = use_left ? lbranch : rbranch
        leave = use_left ? lbest : rbest
        leave_index = findlast(==(leave), branch)
        reunion_segment = reverse(branch[leave_index:end-1])
        append_shared!(left, right, reunion_segment)
        steps += 2length(reunion_segment)
        current = leave
        push!(leaves, leave)

        # BUG1 reachability test: the departure ray re-enters the same obstacle.
        departure = line_step(current, target)
        departure_cell = current + departure
        if departure_cell in observed_obstacle
            return PBUG1Result(:unreachable, left, right, hits, leaves, iteration)
        end
        departure == Cell(0, 0) && return PBUG1Result(
            :reached, left, right, hits, leaves, iteration)
    end

    return PBUG1Result(:step_limit, left, right, hits, leaves, iteration)
end

end # module
