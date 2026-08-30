# include("pbug1.jl")
# using .PBUG1

# obstacles = falses(100, 100)
# obstacles[40:60, 45:55] .= true

# world = GridWorld(obstacles)

# result = pbug1(
#     world,
#     Cell(50, 10),
#     Cell(50, 90)
# )

# println(result.status)
# println(length(result.left_path))
# println(length(result.right_path))
# println(result.hit_points)
# println(result.leave_points)

include("pbug1.jl")
import .PBUG1

include("pbug1_visualization.jl")
import .PBUG1Visualization

using FileIO, Images

include("environments/utils/utilities.jl")


# function png_to_occ_bool(path; thresh=0.5)
#     img = load(path)
#     #small = imresize(img, ratio=0.25)     # 25% of width/height (→ 16% pixels)
#     g = Gray.(img)
#     occ = Float32.(g) .< thresh                       # black -> occupied
#     return occ
# end

# obstacle = falses(15, 15)
# obstacle[6:10, 7:9] .= true

# world = PBUG1.GridWorld(obstacle)
# start = PBUG1.Cell(8, 2)
# target = PBUG1.Cell(8, 14)
const TYPE_I = ["convex", "maze", "office"]

env_data = load_type1_envs()

keys_sorted = sort(collect(keys(env_data)); by=key -> (key[1], key[2], key[3]))
tests = [
    (key[1], key[2], key[3], env_data[key])
    for key in keys_sorted
    if key[1] in TYPE_I
]

for test in tests
    env_class, env_num, pair_num, environment = test
    world = PBUG1.GridWorld(environment["map"])
    start = PBUG1.Cell(environment["start"]...)
    target = PBUG1.Cell(environment["goal"]...)
    
    result = PBUG1.pbug1(world, start, target)

    PBUG1Visualization.save_pbug1_plot(joinpath("results","$(env_class)_$(env_num)_$(pair_num).png"),
        world,
        result,
        target
    )
end

# occ = png_to_occ_bool("environment.png")
# world = PBUG1.GridWorld(occ)
# nrows, _ = size(occ)
# start = PBUG1.Cell(nrows - 26, 92 + 1) # (x,y), 0-based -> (row,col), 1-based
# target = PBUG1.Cell(nrows - 87, 53 + 1)


# result = PBUG1.pbug1(world, start, target)

# # plt = PBUG1Visualization.plot_pbug1(
# #     world,
# #     result,
# #     target
# # )

# # display(plt)

# PBUG1Visualization.save_pbug1_plot(
#     "pbug1_paths.png",
#     world,
#     result,
#     target
# )