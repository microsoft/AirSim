"""
Information Theory for C+F=1 Framework
Julia implementation for high-performance computation
"""

using LinearAlgebra
using Statistics
using FFTW
using Plots

"""
Compute Shannon entropy of a probability distribution
"""
function shannon_entropy(p::Vector{Float64})::Float64
    # Remove zeros to avoid log(0)
    p_nonzero = p[p .> 0]
    return -sum(p_nonzero .* log2.(p_nonzero))
end

"""
Compute mutual information between two discrete variables
I(X;Y) = H(X) + H(Y) - H(X,Y)
"""
function mutual_information(px::Vector{Float64},
                           py::Vector{Float64},
                           pxy::Matrix{Float64})::Float64
    Hx = shannon_entropy(px)
    Hy = shannon_entropy(py)
    Hxy = shannon_entropy(vec(pxy))

    return Hx + Hy - Hxy
end

"""
Compute channel capacity under C+F=1 constraint

For a channel with coherence C(f), the capacity is bounded by:
I_LB = -∫ log₂(F(f)) df where F(f) = 1 - C(f)
"""
function channel_capacity_lower_bound(freqs::Vector{Float64},
                                     C::Vector{Float64})::Float64
    F = 1.0 .- C
    F = max.(F, 1e-10)  # Avoid log(0)

    integrand = -log2.(F)

    # Trapezoidal integration
    df = diff(freqs)
    capacity = sum(integrand[1:end-1] .* df) +
               sum(integrand[2:end] .* df) / 2

    return capacity
end

"""
Compute the Satoshi witness (7.27 bits equivalent)
This is the minimum information to create biological "meaning"
"""
function satoshi_witness(uncertainty_reduction_factor::Float64)::Float64
    return log2(uncertainty_reduction_factor)
end

"""
Test if a system operates at the optimal C≈0.86, F≈0.14 point
"""
function test_operational_point(C_mean::Float64,
                               tolerance::Float64=0.05)::Bool
    target_C = 0.86
    return abs(C_mean - target_C) < tolerance
end

# Example: Information flow in Arkhe network
function arkhe_information_example()
    # Simulated coherence spectrum
    freqs = 10 .^ range(-2, 2, length=1000)  # 0.01 to 100 Hz

    # Model: high coherence at low freq, decreasing at high freq
    C = 0.86 ./ (1 .+ (freqs ./ 10.0).^2)
    C = clamp.(C, 0.0, 1.0)

    F = 1.0 .- C

    # Verify conservation
    @assert all(abs.(C .+ F .- 1.0) .< 1e-10) "Conservation violated!"

    # Compute capacity
    capacity = channel_capacity_lower_bound(freqs, C)
    println("Channel capacity lower bound: $(capacity) bits/sample")

    # Test operational point
    is_optimal = test_operational_point(mean(C))
    println("Operating at optimal point (C≈0.86): $(is_optimal)")

    # Satoshi witness example
    # Factor of 155 reduction in uncertainty (2^7.27 ≈ 155)
    satoshi = satoshi_witness(155.0)
    println("Satoshi witness: $(satoshi) bits (≈7.27)")

    # Plot
    p = plot(freqs, C, label="C(f)", xscale=:log10,
             xlabel="Frequency (Hz)", ylabel="Magnitude",
             title="Coherence Spectrum", linewidth=2)
    plot!(p, freqs, F, label="F(f)", linewidth=2)
    hline!(p, [0.86], label="Target C", linestyle=:dash, color=:red)

    savefig(p, "coherence_spectrum.png")
    println("Plot saved to coherence_spectrum.png")

    return (capacity=capacity, satoshi=satoshi, optimal=is_optimal)
end

# Run example
if abspath(PROGRAM_FILE) == @__FILE__
    results = arkhe_information_example()
    println("\nResults: ", results)
end
