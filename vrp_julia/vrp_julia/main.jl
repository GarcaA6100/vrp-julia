"""
main.jl — Run the VRP solver on sample data or a custom CSV file.

Usage:
    julia main.jl                     # runs built-in demo
    julia main.jl data/my_instance.csv
"""

include("src/VehicleRoutingProblem.jl")
using .VehicleRoutingProblem: Customer, Depot, VRPInstance, VRPSolution,
    generate_random_instance, load_instance, solve_vrp,
    print_solution, save_results, build_distance_matrix

function run_demo()
    println("\n Vehicle Routing Problem Solver")
    println("    Julia | Optimization | Industry Demo\n")

    # ── Demo 1: Small hand-crafted instance ──────────────────────────
    println("━"^58)
    println("DEMO 1 — Small Instance (8 customers, 3 vehicles)")
    println("━"^58)

    small = VRPInstance(
        [
            Customer(1,  10.0, 20.0, 15.0, "Warehouse A"),
            Customer(2,  30.0, 60.0, 20.0, "Mall B"),
            Customer(3,  50.0, 80.0, 10.0, "Office C"),
            Customer(4,  70.0, 40.0, 25.0, "Hotel D"),
            Customer(5,  85.0, 15.0, 18.0, "School E"),
            Customer(6,  60.0, 10.0, 12.0, "Hospital F"),
            Customer(7,  20.0, 75.0, 22.0, "Airport G"),
            Customer(8,  45.0, 45.0,  8.0, "City Hall H"),
        ],
        Depot(50.0, 50.0),
        3,       # vehicles
        70.0,    # capacity per vehicle
        "Small City Demo"
    )

    sol_small, D_small = solve_vrp(small; verbose=true)
    print_solution(sol_small, D_small)
    save_results(sol_small, D_small, "results/small")

    # ── Demo 2: Random medium instance ───────────────────────────────
    println("\n" * "━"^58)
    println("DEMO 2 — Medium Random Instance (20 customers, 4 vehicles)")
    println("━"^58)

    medium = generate_random_instance(20; seed=7, num_vehicles=4, capacity=120.0,
                                       name="Medium Random Instance")
    sol_med, D_med = solve_vrp(medium; verbose=true)
    print_solution(sol_med, D_med)
    save_results(sol_med, D_med, "results/medium")

    # ── Demo 3: Load from CSV if provided ────────────────────────────
    if length(ARGS) >= 1 && isfile(ARGS[1])
        println("\n" * "━"^58)
        println("DEMO 3 — Custom CSV: $(ARGS[1])")
        println("━"^58)
        custom = load_instance(ARGS[1]; num_vehicles=5, capacity=100.0, name="Custom Instance")
        sol_c, D_c = solve_vrp(custom; verbose=true)
        print_solution(sol_c, D_c)
        save_results(sol_c, D_c, "results/custom")
    elseif length(ARGS) >= 1
        println("\n File not found: $(ARGS[1])")
    end

    println("\n All demos complete. Check the results/ folder for CSV outputs.\n")
end

run_demo()
