# Vehicle Routing Problem — Julia Project

This project solves the **Vehicle Routing Problem (VRP)** using Julia. The idea is pretty simple: you have a strong customer base that need deliveries, a fleet of vehicles, and each vehicle can only carry so much. The goal is to figure out the most efficient routes so every customer gets their delivery without overloading any vehicle.

---

## What It Does

Given a list of customers with locations and delivery demands, the solver:

- Assigns customers to vehicles
- Makes sure no vehicle carries more than its capacity
- Finds routes that start and end at a central depot
- Tries to minimize the total distance traveled

---

## How It Works (Algorithm)

It runs three steps back to back:

```
1. Build a distance matrix between all locations
        ↓
2. Nearest-Neighbor Heuristic (construction)
   → Greedily builds routes by always going to the closest customer that fits
        ↓
3. 2-opt Local Search (improvement)
   → Looks for ways to "uncross" routes and shorten them
        ↓
4. Or-opt Relocation (improvement)
   → Tries moving individual customers to better routes
        ↓
5. Save results to CSV files
```

---

## Project Structure

```
vrp_julia/
├── src/
│   └── VehicleRoutingProblem.jl   # All the core logic lives here
├── test/
│   └── runtests.jl                # Tests to make sure everything works
├── data/
│   └── sample_instance.csv        # A sample dataset to test with
├── results/                       # Gets created automatically when you run it
│   ├── small/
│   └── medium/
├── main.jl                        # This is what you run
└── README.md
```

---

## How to Run It

First make sure you have Julia installed from [julialang.org](https://julialang.org/downloads/).

Then in your terminal:

```bash
julia main.jl
```

That runs two demos automatically — a small 8-customer one and a medium 20-customer one.

You can also run it on your own data:

```bash
julia main.jl data/sample_instance.csv
```

To run the tests:

```bash
julia test/runtests.jl
```

---

## Using Your Own Data

If you want to test it with your own delivery locations, just make a CSV file like this:

```
id,x,y,demand,label
0,50.0,50.0,0.0,Central Depot
1,12.5,78.3,18.0,Customer A
2,25.0,91.0,12.5,Customer B
```

- Row `id=0` is always the depot
- `x` and `y` are the coordinates
- `demand` is how much that customer needs delivered
- `label` is just a name so you can identify them in the output

---

## Output

After running, it saves two CSV files in the `results/` folder:

**routes.csv** — shows every stop for every vehicle, including how much load and distance was accumulated along the way

**summary.csv** — a quick overview per vehicle: how many stops, total load, capacity used, and total distance

---

## Sample Output

```
══════════════════════════════════════════════════════════
  VEHICLE ROUTING PROBLEM — SOLUTION REPORT
  Small City Demo
══════════════════════════════════════════════════════════

  Vehicles used : 2 / 3
  Total distance: 243.18 km
  Feasible      : ✓ Yes

  Vehicle 1
    Route    : Depot → Mall B → Airport G → Warehouse A → Depot
    Stops    : 3 customers
    Load     : 60.0 / 70.0 (86%)
    Distance : 124.51 km

  Vehicle 2
    Route    : Depot → Office C → Hotel D → School E → Depot
    Stops    : 3 customers
    Load     : 55.0 / 70.0 (79%)
    Distance : 118.67 km
══════════════════════════════════════════════════════════
```

---

## What Could Be Added Next

Some ideas to take this further:

- Add time windows so customers have to be visited before a certain time
- Support different vehicle sizes with different costs
- Try a genetic algorithm for bigger instances
- Plot the routes on an actual map

---

## License

MIT — free to use and modify.
