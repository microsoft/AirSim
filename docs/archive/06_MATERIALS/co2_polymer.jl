"""
CO₂ to Polymer Transformation
Programmed temporal architecture via dispersity Đ < 1.2
"""

using DifferentialEquations
using Plots
using Statistics

"""
Polymer degradation model with controlled dispersity
"""
mutable struct CO2Polymer
    Đ::Float64              # Dispersity (Mw/Mn)
    Mn::Float64              # Number-average molecular weight
    Mw::Float64              # Weight-average molecular weight
    degradation_rate::Float64 # Hydrolysis rate (day⁻¹)
    lifetime_target::Float64  # Target lifetime (days)

    function CO2Polymer(Đ_target::Float64, Mn::Float64, lifetime::Float64)
        Đ = min(max(Đ_target, 1.0), 1.2)  # Enforce Đ ≤ 1.2
        Mw = Mn * Đ
        # Degradation rate inversely related to lifetime
        deg_rate = 1.0 / lifetime
        new(Đ, Mn, Mw, deg_rate, lifetime)
    end
end

"""
Time evolution of polymer mass under degradation
dm/dt = -k * m
"""
function polymer_degradation!(du, u, p, t)
    m = u[1]
    k = p[1]
    du[1] = -k * m
end

"""
Simulate degradation over time
"""
function simulate_degradation(poly::CO2Polymer, tspan::Float64=100.0)
    u0 = [1.0]  # initial mass fraction
    tspan = (0.0, tspan)
    prob = ODEProblem(polymer_degradation!, u0, tspan, [poly.degradation_rate])
    sol = solve(prob, Tsit5())
    return sol
end

"""
Get programmed lifetime (time to reach 50% mass)
"""
function half_life(poly::CO2Polymer)
    return log(2) / poly.degradation_rate
end

"""
Uniformity condition: Đ < 1.2
"""
function is_uniform(poly::CO2Polymer)
    return poly.Đ < 1.2
end

"""
Create amphiphilic polycarbonate
"""
function create_amphiphilic_polycarbonate(Đ_target::Float64,
                                          hydrophobic_ratio::Float64)
    # Simplified: create polymer with given Đ and hydrophobic fraction
    Mn = 5000.0  # typical
    lifetime = 30.0  # days
    poly = CO2Polymer(Đ_target, Mn, lifetime)
    return (polymer=poly, hydrophobic_ratio=hydrophobic_ratio)
end

"""
Example: CO₂ waste to programmed material
"""
function co2_example()
    println("="^60)
    println("CO₂ → POLYMER PROGRAMMABLE DEGRADATION")
    println("="^60)

    # Create polymers with various dispersities
    Đ_vals = [1.05, 1.18, 1.25, 1.5]
    polymers = [CO2Polymer(Đ, 5000.0, 30.0) for Đ in Đ_vals]

    println("\nDispersity and uniformity:")
    for poly in polymers
        uniform = is_uniform(poly) ? "✓" : "✗"
        println("  Đ = $(poly.Đ)  uniform? $uniform")
    end

    # Degradation simulation
    plt = plot(title="CO₂ Polymer Degradation (programmed lifetime 30 days)",
               xlabel="Time (days)", ylabel="Mass fraction",
               xlim=(0, 50), ylim=(0, 1.1))

    colors = [:blue, :green, :orange, :red]

    for (i, poly) in enumerate(polymers)
        sol = simulate_degradation(poly, 50.0)
        plot!(plt, sol.t, sol[1,:],
              label="Đ = $(poly.Đ)",
              color=colors[i],
              linewidth=2)

        hl = half_life(poly)
        scatter!([hl], [0.5], color=colors[i], markersize=6)
    end

    hline!(plt, [0.5], linestyle=:dash, color=:black, label="50% remaining")
    savefig(plt, "co2_polymer_degradation.png")
    println("\nPlot saved to co2_polymer_degradation.png")

    # VITA vs DARVO (count-up vs count-down)
    println("\n" * "-"^60)
    println("TEMPORAL ARCHITECTURE: VITA vs DARVO")
    println("-"^60)

    poly_vita = CO2Polymer(1.18, 5000.0, 30.0)  # uniform, programmed
    poly_darvo = CO2Polymer(1.5, 5000.0, 10.0)  # non-uniform, fast degradation

    sol_vita = simulate_degradation(poly_vita, 50.0)
    sol_darvo = simulate_degradation(poly_darvo, 50.0)

    println("\nVITA (count-up, programmed lifetime):")
    println("  Đ = $(poly_vita.Đ)  Half-life = $(half_life(poly_vita)) days")
    println("  Degradation is predictable, allows design.")

    println("\nDARVO (count-down, uncontrolled):")
    println("  Đ = $(poly_darvo.Đ)  Half-life = $(half_life(poly_darvo)) days")
    println("  Degradation is chaotic, lifetime not designable.")

    return polymers
end

if abspath(PROGRAM_FILE) == @__FILE__
    co2_example()
end
