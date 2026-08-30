using FileIO, Images

# --------------------------------------------------
# Converting PNG images to occupancy grids (boolean arrays).
# --------------------------------------------------

function png_to_occ_bool(path; thresh=0.5)
    img = load(path)
    #small = imresize(img, ratio=0.25)     # 25% of width/height (→ 16% pixels)
    g = Gray.(img)
    occ = Float32.(g) .< thresh                       # black -> occupied
    return occ
end

# --------------------------------------------------
# Extracting and retrieving type1 environments.
# --------------------------------------------------

const TYPE_I = ["convex", "maze", "office"]

function load_type1_envs()
    current_dir = @__DIR__
    # println("Current directory: $current_dir")
    environments_path = dirname(@__DIR__)
    type1_path = joinpath(environments_path, "type1")
    type1_zip = joinpath(current_dir, "type1.zip")
    if !isdir(type1_path)
        isfile(type1_zip) || error("Dataset archive not found: $type1_zip")
        run(`unzip -q -o $type1_zip -d $environments_path`)
    end

    isdir(type1_path) || error("Dataset directory not found: $type1_path")

    env_data = Dict{Tuple{String,Int,Int}, Dict{String,Any}}()
    
    for folder in sort(readdir(type1_path; join=true))
        isdir(folder) || continue

        joined_name_options = join(TYPE_I, "|") 
        pattern = Regex("^($joined_name_options)_(\\d+)\$")
        
        match_result = match(pattern, basename(folder))        
        match_result === nothing && continue

        env_class = match_result.captures[1]
        env_index_in_class = parse(Int, match_result.captures[2])

        map_path = joinpath(folder, "environment.png")
        pairs_path = joinpath(folder, "pairs.txt")
        isfile(map_path) || error("Missing map: $map_path")
        isfile(pairs_path) || error("Missing pairs file: $pairs_path")

        map = png_to_occ_bool(map_path; thresh=1.0)
        lines = readlines(pairs_path)

        for (pair_index, line) in enumerate(lines)
            nums = parse.(Int, split(line))

            nrows, _ = size(map)
            start = (nrows - nums[2], nums[1] + 1) # (x,y), 0-based -> (row,col), 1-based
            goal = (nrows - nums[4], nums[3] + 1)

            env_data[(env_class, env_index_in_class, pair_index)] = Dict(
                "map" => map,
                "start" => start,
                "goal" => goal,
            )
        end
    end

    return env_data
end

function get_type1_env(env_class::String, env_index_in_class::Int, pair_index::Int)
    if !(env_class in TYPE_I)
        error("Invalid environment class: $env_class")
    end
    env_data = load_type1_envs()
    key = (env_class, env_index_in_class, pair_index)
    haskey(env_data, key) || error("Environment not found: $key")
    return env_data[key]
end

# --------------------------------------------------
# Extracting and retrieving type2 environments.
# --------------------------------------------------

const TYPE_II = ["rimon", "dcomb", "loops", "marina", "cds", "warehouse", "garage", "terminal", "courtyard", "platforms", "culdesac", "doorways", "bridges", "chevrons", "splitroom", "busterminal", "parallelrows", "nestedsplitroom"]

function read_type2_pair(pair_path::AbstractString)
    nums = parse.(Int, split(strip(read(pair_path, String))))
    length(nums) == 4 || error(
        "Expected four integers in $pair_path: start_row start_col goal_row goal_col",
    )

    start = (nums[1], nums[2])
    goal = (nums[3], nums[4])
    return start, goal
end

function load_type2_envs()
    current_dir = @__DIR__
    environments_path = dirname(@__DIR__)
    type2_path = joinpath(environments_path, "type2")

    if !isdir(type2_path)
        run(`julia $(joinpath(current_dir, "create_type2_environments.jl"))`)
    end

    isdir(type2_path) || error("Dataset directory not found: $type2_path")

    env_data = Dict{Tuple{String,Int}, Dict{String,Any}}()

    for folder in sort(readdir(type2_path; join=true))
        isdir(folder) || continue

        for map_path in sort(readdir(folder; join=true))
            isfile(map_path) || continue
            endswith(lowercase(map_path), ".png") || continue

            joined_name_options = join(TYPE_II, "|") 
            pattern = Regex("^($joined_name_options)_(\\d+)\$")
            
            match_result = match(pattern, basename(map_path)[1:end-4])  # Remove ".png" extension   
            match_result === nothing && continue

            env_class = match_result.captures[1]
            env_index_in_class = parse(Int, match_result.captures[2])

            if !(env_class in TYPE_II)
                error("Invalid environment class: $env_class")
            end

            map = png_to_occ_bool(map_path; thresh=1.0)

            pair_path = joinpath(folder, "pair_$(env_index_in_class).txt")
            
            if isfile(pair_path)
                start, goal = read_type2_pair(pair_path)
            else
                error("Missing pair file: $pair_path")
            end

            nrows, ncols = size(map)
            1 <= start[1] <= nrows && 1 <= start[2] <= ncols ||
                error("Start $start is outside map $map_path")
            1 <= goal[1] <= nrows && 1 <= goal[2] <= ncols ||
                error("Goal $goal is outside map $map_path")
            !map[start...] || error("Start $start is occupied in $map_path")
            !map[goal...] || error("Goal $goal is occupied in $map_path")

            env_data[(env_class, env_index_in_class)] = Dict(
                    "map" => map,
                    "start" => start,
                    "goal" => goal,
                )
        end

    end

    return env_data
end
