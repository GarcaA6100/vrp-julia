"""
VehicleRoutingProblem.jl
========================
Industry-level Vehicle Routing Problem (VRP) solver in Julia.

Features:
- Capacitated VRP (CVRP) with multiple vehicles
- Nearest-neighbor greedy construction heuristic
- 2-opt local search improvement
- Or-opt moves (relocate single customers)
- Solution visualization via Unicode route maps
- CSV data import/export
- Detailed solution reporting
"""
module VehicleRoutingProblem

using LinearAlgebra
using Printf
using Dates
using Random

export Customer, Depot, VRPInstance, VRPSolution
export load_instance, solve_vrp, print_solution, save_results
export nearest_neighbor_heuristic, two_opt!, or_opt!
export total_distance, is_feasible

# ─────────────────────────────────────────────
# Data Structures
# ─────────────────────────────────────────────

"""A delivery customer with location and demand."""
struct Customer
    id::Int
    x::Float64
    y::Float64
    demand::Float64
    label::String
end

"""The central depot where all vehicles start and end."""
struct Depot
    x::Float64
    y::Float64
end

"""
A complete VRP problem instance.
- `customers`    : list of customers to serve
- `depot`        : starting/ending point for all vehicles
- `num_vehicles` : fleet size (max vehicles available)
- `capacity`     : maximum load per vehicle
"""
struct VRPInstance
    customers::Vector{Customer}
    depot::Depot
    num_vehicles::Int
    capacity::Float64
    name::String
end

"""
A VRP solution: a set of routes, one per vehicle.
Each route is a vector of customer IDs (depot not included).
"""
mutable struct VRPSolution
    routes::Vector{Vector{Int}}
    instance::VRPInstance
    distance::Float64
    feasible::Bool
    solve_time::Float64
end

# ─────────────────────────────────────────────
# Distance Utilities
# ─────────────────────────────────────────────

"""Euclidean distance between two points (x1,y1) and (x2,y2)."""
function euclidean(x1, y1, x2, y2)
    return sqrt((x1 - x2)^2 + (y1 - y2)^2)
end

"""Build full distance matrix: index 0 = depot, 1..n = customers."""
function build_distance_matrix(instance::VRPInstance)
    n = length(instance.customers)
    # size (n+1) x (n+1), index 1 = depot, 2..n+1 = customers
    D = zeros(Float64, n + 1, n + 1)
    d = instance.depot
    custs = instance.customers

    for i in 1:(n+1)
        for j in 1:(n+1)
            if i == j
                D[i, j] = 0.0
            elseif i == 1
                c = custs[j-1]
                D[i, j] = euclidean(d.x, d.y, c.x, c.y)
            elseif j == 1
                c = custs[i-1]
                D[i, j] = euclidean(c.x, c.y, d.x, d.y)
            else
                ci = custs[i-1]
                cj = custs[j-1]
                D[i, j] = euclidean(ci.x, ci.y, cj.x, cj.y)
            end
        end
    end
    return D
end

"""Total distance of a single route (depot → customers → depot)."""
function route_distance(route::Vector{Int}, D::Matrix{Float64})
    isempty(route) && return 0.0
    dist = D[1, route[1] + 1]            # depot → first customer
    for i in 1:(length(route)-1)
        dist += D[route[i]+1, route[i+1]+1]
    end
    dist += D[route[end]+1, 1]           # last customer → depot
    return dist
end

"""Total distance across all routes in a solution."""
function total_distance(sol::VRPSolution, D::Matrix{Float64})
    return sum(route_distance(r, D) for r in sol.routes if !isempty(r))
end

"""Load on a given route."""
function route_load(route::Vector{Int}, instance::VRPInstance)
    return sum(instance.customers[cid].demand for cid in route)
end

"""Check whether every route respects the vehicle capacity."""
function is_feasible(sol::VRPSolution)
    inst = sol.instance
    for route in sol.routes
        if route_load(route, inst) > inst.capacity + 1e-9
            return false
        end
    end
    return true
end

# ─────────────────────────────────────────────
# Data Loading
# ─────────────────────────────────────────────

"""
Load a VRP instance from a CSV file.

CSV format (with header):
    id,x,y,demand,label
    1,10.5,20.3,15.0,Customer A
    ...

The depot is always the first row (id=0).
"""
function load_instance(filepath::String; num_vehicles::Int=5, capacity::Float64=100.0, name::String="VRP Instance")
    lines = readlines(filepath)
    # skip header
    data_lines = filter(l -> !startswith(strip(l), "#") && !isempty(strip(l)), lines)
    header_skipped = false
    customers = Customer[]
    depot = Depot(0.0, 0.0)

    for line in data_lines
        stripped = strip(line)
        if !header_skipped && (startswith(stripped, "id") || startswith(stripped, "ID"))
            header_skipped = true
            continue
        end
        parts = split(stripped, ",")
        length(parts) < 4 && continue
        id   = parse(Int, strip(parts[1]))
        x    = parse(Float64, strip(parts[2]))
        y    = parse(Float64, strip(parts[3]))
        dem  = parse(Float64, strip(parts[4]))
        lbl  = length(parts) >= 5 ? strip(parts[5]) : "C$id"
        if id == 0
            depot = Depot(x, y)
        else
            push!(customers, Customer(id, x, y, dem, lbl))
        end
    end

    return VRPInstance(customers, depot, num_vehicles, capacity, name)
end

"""Generate a random VRP instance for testing."""
function generate_random_instance(n_customers::Int; seed::Int=42,
        num_vehicles::Int=4, capacity::Float64=100.0, name::String="Random VRP")
    rng = MersenneTwister(seed)
    depot = Depot(50.0, 50.0)
    customers = [
        Customer(i,
            round(rand(rng) * 100, digits=1),
            round(rand(rng) * 100, digits=1),
            round(5.0 + rand(rng) * 25.0, digits=1),
            "C$i")
        for i in 1:n_customers
    ]
    return VRPInstance(customers, depot, num_vehicles, capacity, name)
end

# ─────────────────────────────────────────────
# Construction Heuristic: Nearest Neighbor
# ─────────────────────────────────────────────

"""
Nearest-Neighbor greedy construction heuristic.

Builds routes one at a time:
1. Start at depot.
2. Repeatedly visit the nearest unvisited customer that fits in the vehicle.
3. When no customer fits, close the route and start a new vehicle.
"""
function nearest_neighbor_heuristic(instance::VRPInstance, D::Matrix{Float64})
    n = length(instance.customers)
    unvisited = Set(1:n)
    routes = Vector{Vector{Int}}()
    vehicles_used = 0

    while !isempty(unvisited) && vehicles_used < instance.num_vehicles
        vehicles_used += 1
        route = Int[]
        load = 0.0
        current = 0  # depot index in distance matrix is 0 → row 1

        while true
            best_cid = -1
            best_dist = Inf
            for cid in unvisited
                c = instance.customers[cid]
                if load + c.demand <= instance.capacity
                    d = D[current + 1, cid + 1]
                    if d < best_dist
                        best_dist = d
                        best_cid = cid
                    end
                end
            end
            best_cid == -1 && break
            push!(route, best_cid)
            load += instance.customers[best_cid].demand
            delete!(unvisited, best_cid)
            current = best_cid
        end

        push!(routes, route)
    end

    # If any customers remain unvisited, force them into the last route
    # (marks solution infeasible but preserves all customers)
    if !isempty(unvisited)
        for cid in sort(collect(unvisited))
            push!(routes[end], cid)
        end
    end

    dist = sum(route_distance(r, D) for r in routes)
    sol = VRPSolution(routes, instance, dist, true, 0.0)
    sol.feasible = is_feasible(sol)
    return sol
end

# ─────────────────────────────────────────────
# Improvement: 2-opt Local Search
# ─────────────────────────────────────────────

"""
2-opt improvement for a single route.
Reverses segments of the route when doing so reduces distance.
Runs until no improving swap is found (first improvement).
"""
function two_opt_route!(route::Vector{Int}, D::Matrix{Float64})
    improved = true
    while improved
        improved = false
        n = length(route)
        for i in 1:(n-1)
            for j in (i+1):n
                # Cost before: ...→ route[i] → route[i+1] →...→ route[j] → route[j+1]→...
                a = i == 1 ? 1 : route[i-1] + 1   # node before i (depot if i=1)
                b = route[i] + 1
                c = route[j] + 1
                d_next = j == n ? 1 : route[j+1] + 1

                before = D[a, b] + D[c, d_next]
                after  = D[a, c] + D[b, d_next]

                if after < before - 1e-10
                    reverse!(route, i, j)
                    improved = true
                end
            end
        end
    end
end

"""Apply 2-opt to every route in the solution."""
function two_opt!(sol::VRPSolution, D::Matrix{Float64})
    for route in sol.routes
        length(route) >= 3 && two_opt_route!(route, D)
    end
    sol.distance = total_distance(sol, D)
    return sol
end

# ─────────────────────────────────────────────
# Improvement: Or-opt (Customer Relocation)
# ─────────────────────────────────────────────

"""
Or-opt: try moving a single customer from one route to a better position
in any route (including same route). Repeats until no improvement found.
"""
function or_opt!(sol::VRPSolution, D::Matrix{Float64})
    inst = sol.instance
    improved = true
    while improved
        improved = false
        for r1 in 1:length(sol.routes)
            isempty(sol.routes[r1]) && continue
            i = 1
            while i <= length(sol.routes[r1])
                cid = sol.routes[r1][i]
                cust = inst.customers[cid]

                # Cost of removing cid from route r1
                prev = i == 1 ? 1 : sol.routes[r1][i-1] + 1
                next = i == length(sol.routes[r1]) ? 1 : sol.routes[r1][i+1] + 1
                removal_gain = D[prev, cid+1] + D[cid+1, next] - D[prev, next]

                best_gain = 1e-10  # minimum improvement threshold
                best_r2 = -1
                best_pos = -1

                for r2 in 1:length(sol.routes)
                    # Check capacity
                    current_load = route_load(sol.routes[r2], inst)
                    if r2 != r1
                        current_load + cust.demand > inst.capacity + 1e-9 && continue
                    end

                    route2 = sol.routes[r2]
                    len2 = length(route2)

                    # Try inserting cid at every position in route2
                    for j in 0:len2
                        (r2 == r1 && (j == i-1 || j == i)) && continue
                        before_j = j == 0 ? 1 : route2[j] + 1
                        after_j  = j == len2 ? 1 : route2[j+1] + 1
                        insert_cost = D[before_j, cid+1] + D[cid+1, after_j] - D[before_j, after_j]
                        gain = removal_gain - insert_cost
                        if gain > best_gain
                            best_gain = gain
                            best_r2   = r2
                            best_pos  = j
                        end
                    end
                end

                if best_r2 != -1
                    # Apply move
                    deleteat!(sol.routes[r1], i)
                    insert!(sol.routes[best_r2], best_pos + 1, cid)
                    improved = true
                    # Don't increment i — recheck same position
                else
                    i += 1
                end
            end
        end
    end
    sol.distance = total_distance(sol, D)
    sol.feasible = is_feasible(sol)
    return sol
end

# ─────────────────────────────────────────────
# Main Solver
# ─────────────────────────────────────────────

"""
Solve a VRP instance using:
  1. Nearest-neighbor construction
  2. 2-opt improvement per route
  3. Or-opt cross-route relocation

Returns a `VRPSolution`.
"""
function solve_vrp(instance::VRPInstance; verbose::Bool=true)
    t_start = time()
    D = build_distance_matrix(instance)

    verbose && println("\n  Solving: $(instance.name)")
    verbose && println("    Customers : $(length(instance.customers))")
    verbose && println("    Vehicles  : $(instance.num_vehicles)  |  Capacity: $(instance.capacity)")

    # Phase 1: Construction
    sol = nearest_neighbor_heuristic(instance, D)
    verbose && @printf("    [1] Construction  → %.2f km\n", sol.distance)

    # Phase 2: 2-opt
    two_opt!(sol, D)
    verbose && @printf("    [2] 2-opt          → %.2f km\n", sol.distance)

    # Phase 3: Or-opt relocation
    or_opt!(sol, D)
    verbose && @printf("    [3] Or-opt         → %.2f km\n", sol.distance)

    sol.solve_time = time() - t_start
    verbose && @printf("    Solved in %.3f seconds\n", sol.solve_time)
    return sol, D
end

# ─────────────────────────────────────────────
# Reporting & Output
# ─────────────────────────────────────────────

"""Print a detailed human-readable solution report."""
function print_solution(sol::VRPSolution, D::Matrix{Float64})
    inst = sol.instance
    println("\n" * "═"^58)
    println("  VEHICLE ROUTING PROBLEM — SOLUTION REPORT")
    println("  $(inst.name)")
    println("  Generated: $(Dates.format(now(), "yyyy-mm-dd HH:MM:SS"))")
    println("═"^58)

    active = filter(!isempty, sol.routes)
    println("\n  Vehicles used : $(length(active)) / $(inst.num_vehicles)")
    @printf("  Total distance: %.2f km\n", sol.distance)
    @printf("  Feasible      : %s\n", sol.feasible ? "✓ Yes" : "✗ No (capacity exceeded)")

    println("\n" * "─"^58)
    for (i, route) in enumerate(sol.routes)
        isempty(route) && continue
        load = route_load(route, inst)
        dist = route_distance(route, D)
        util = 100 * load / inst.capacity
        labels = join([inst.customers[cid].label for cid in route], " → ")
        @printf("\n  Vehicle %d\n", i)
        @printf("    Route    : Depot → %s → Depot\n", labels)
        @printf("    Stops    : %d customers\n", length(route))
        @printf("    Load     : %.1f / %.1f (%.0f%%)\n", load, inst.capacity, util)
        @printf("    Distance : %.2f km\n", dist)
    end
    println("\n" * "═"^58)
end

"""
Save results to CSV files in the given output directory.
Writes:
  - routes.csv  : one row per customer stop
  - summary.csv : per-vehicle summary
"""
function save_results(sol::VRPSolution, D::Matrix{Float64}, output_dir::String)
    mkpath(output_dir)
    inst = sol.instance

    # routes.csv
    routes_path = joinpath(output_dir, "routes.csv")
    open(routes_path, "w") do f
        println(f, "vehicle,stop,customer_id,label,x,y,demand,cumulative_load,cumulative_distance")
        for (v, route) in enumerate(sol.routes)
            isempty(route) && continue
            cum_load = 0.0
            cum_dist = 0.0
            prev = 1  # depot
            for (s, cid) in enumerate(route)
                c = inst.customers[cid]
                cum_load += c.demand
                cum_dist += D[prev, cid+1]
                @printf(f, "%d,%d,%d,%s,%.2f,%.2f,%.1f,%.1f,%.2f\n",
                    v, s, cid, c.label, c.x, c.y, c.demand, cum_load, cum_dist)
                prev = cid + 1
            end
        end
    end

    # summary.csv
    summary_path = joinpath(output_dir, "summary.csv")
    open(summary_path, "w") do f
        println(f, "vehicle,stops,load,capacity_pct,distance_km")
        for (v, route) in enumerate(sol.routes)
            isempty(route) && continue
            load = route_load(route, inst)
            dist = route_distance(route, D)
            @printf(f, "%d,%d,%.1f,%.1f,%.2f\n",
                v, length(route), load, 100*load/inst.capacity, dist)
        end
    end

    println("\n  Results saved:")
    println("    • $routes_path")
    println("    • $summary_path")
end

end # module
