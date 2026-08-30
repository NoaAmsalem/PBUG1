using Test
include("pbug1.jl")
using .PBUG1

@testset "PBUG1" begin
    empty_world = GridWorld(falses(12, 12))
    r = pbug1(empty_world, Cell(6, 2), Cell(6, 10))
    @test r.status == :reached
    @test last(r.left_path) == Cell(6, 10)
    @test r.left_path == r.right_path

    obstacle = falses(15, 15)
    obstacle[6:10, 7:9] .= true
    r = pbug1(GridWorld(obstacle), Cell(8, 2), Cell(8, 14))
    @test r.status == :reached
    @test !isempty(r.hit_points)
    @test last(r.leave_points) != first(r.leave_points)

    blocked_target = copy(obstacle)
    blocked_target[8, 14] = true
    r = pbug1(GridWorld(blocked_target), Cell(8, 2), Cell(8, 14))
    @test r.status == :unreachable
end
