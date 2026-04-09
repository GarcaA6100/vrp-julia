"""
test/runtests.jl — Unit tests for the VRP solver.
Run with: julia test/runtests.jl
"""

include("../src/VehicleRoutingProblem.jl")
using .VehicleRoutingProblem

# ── Minimal test harness ──────────────────────────────────────────────────────
pass_count = 0
fail_count = 0

macro test(expr, desc)
    quote
        try
            result = $(esc(expr))
            if result
                global pass_count += 1
                println("  ✓  $($desc)")
            else
                global fail_count += 1
                println("  ✗  $($desc)  [FAIL: got false]")
            end
        catch e
            global fail_count += 1
            println("  ✗  $($desc)  [ERROR: $e]")
        end
    end
end

println("\n" * "─"^50)
println("  VRP Solver — Test Suite")
println("─"^50)

# ── Fixtures ──────────────────────────────────────────────────────────────────
tiny = VRPInstance(
    [
        Customer(1, 10.0, 0.0, 20.0, "A"),
        Customer(2, 0.0, 10.0, 20.0, "B"),
        Customer(3, 20.0, 20.0, 20.0, "C"),
        Customer(4, 30.0, 0.0, 20.0, "D"),
    ],
    Depot(0.0, 0.0),
    2,
    50.0,
    "Tiny Test"
)

D = build_distance_matrix(tiny)

# ── Distance Matrix ───────────────────────────────────────────────────────────
println("\n[1] Distance Matrix")
@test size(D) == (5, 5)                         "Matrix is (n+1) × (n+1)"
@test D[1, 1] == 0.0                            "Depot self-distance is 0"
@test D[2, 2] == 0.0                            "Customer self-distance is 0"
@test abs(D[1, 2] - 10.0) < 1e-9               "Depot → Customer 1 = 10.0"
@test abs(D[2, 1] - D[1, 2]) < 1e-9            "Distance matrix is symmetric"

# ── Construction Heuristic ────────────────────────────────────────────────────
println("\n[2] Nearest Neighbor Construction")
sol, _ = solve_vrp(tiny; verbose=false)
all_visited = sort(vcat(sol.routes...))
@test all_visited == [1, 2, 3, 4]               "All customers visited exactly once"
@test length(sol.routes) <= tiny.num_vehicles    "Routes don't exceed vehicle count"
@test sol.distance > 0                           "Total distance is positive"

# ── Feasibility ───────────────────────────────────────────────────────────────
println("\n[3] Feasibility Checking")
@test is_feasible(sol)                           "Solution respects capacity constraints"

# Create an intentionally infeasible solution
bad_sol = VRPSolution([[1, 2, 3, 4]], tiny, 0.0, true, 0.0)
@test !is_feasible(bad_sol)                      "Overloaded route detected as infeasible"

# ── Route Distance ────────────────────────────────────────────────────────────
println("\n[4] Route Distance Calculation")
single_route = [1]  # just customer 1 at (10,0)
dist = route_distance(single_route, D)
# depot(0,0)→C1(10,0) + C1(10,0)→depot(0,0) = 10+10 = 20
@test abs(dist - 20.0) < 1e-9                   "Single-customer route distance correct"

@test route_distance(Int[], D) == 0.0            "Empty route has zero distance"

# ── Load Calculation ──────────────────────────────────────────────────────────
println("\n[5] Route Load")
@test route_load([1, 2], tiny) == 40.0          "Two customers with demand 20 each = 40"
@test route_load(Int[], tiny) == 0.0            "Empty route has zero load"

# ── Random Instance Generation ────────────────────────────────────────────────
println("\n[6] Random Instance Generation")
rand_inst = generate_random_instance(15; seed=1, num_vehicles=3, capacity=90.0)
@test length(rand_inst.customers) == 15          "Correct number of customers generated"
@test rand_inst.num_vehicles == 3                "Correct vehicle count"
@test rand_inst.capacity == 90.0                 "Correct capacity"

rand_sol, rand_D = solve_vrp(rand_inst; verbose=false)
all_rand = sort(vcat(rand_sol.routes...))
@test all_rand == collect(1:15)                  "All 15 random customers visited"
@test is_feasible(rand_sol)                      "Random instance solution is feasible"

# ── Improvement Heuristics ────────────────────────────────────────────────────
println("\n[7] Improvement Heuristics")
med = generate_random_instance(20; seed=42, num_vehicles=4, capacity=120.0)
D_med = build_distance_matrix(med)
nn_sol = nearest_neighbor_heuristic(med, D_med)
nn_dist = nn_sol.distance
two_opt!(nn_sol, D_med)
@test nn_sol.distance <= nn_dist + 1e-9          "2-opt does not worsen solution"
pre_oropt = nn_sol.distance
or_opt!(nn_sol, D_med)
@test nn_sol.distance <= pre_oropt + 1e-9        "Or-opt does not worsen solution"
@test is_feasible(nn_sol)                        "Solution feasible after improvements"

# ── CSV Save & Reload ─────────────────────────────────────────────────────────
println("\n[8] CSV Output")
mkpath("/tmp/vrp_test_out")
save_results(sol, D, "/tmp/vrp_test_out")
@test isfile("/tmp/vrp_test_out/routes.csv")     "routes.csv created"
@test isfile("/tmp/vrp_test_out/summary.csv")    "summary.csv created"

# ── Summary ───────────────────────────────────────────────────────────────────
println("\n" * "─"^50)
println("  Results: $pass_count passed, $fail_count failed")
println("─"^50 * "\n")

if fail_count > 0
    exit(1)
end
