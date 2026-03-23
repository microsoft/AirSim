/**
 * Ordered Water Simulation in Nanoconfinement
 * Role in microtubule quantum coherence
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>

struct WaterMolecule {
    double x, y, z;      // Position (nm)
    double dip_x, dip_y, dip_z;  // Dipole orientation
    double energy;        // Interaction energy
};

class OrderedWater {
private:
    std::vector<WaterMolecule> water;
    double cavity_radius;  // nm
    double cavity_length;  // nm
    double temperature;    // K

    // Physical constants
    const double dipole_strength = 1.85;  // Debye for water
    const double permittivity = 80.0;     // ε for water
    const double kB = 1.380649e-23;       // J/K

    // Random generator
    std::mt19937 gen;
    std::uniform_real_distribution<double> dist;

public:
    OrderedWater(double radius_nm, double length_nm, double temp_K)
        : cavity_radius(radius_nm), cavity_length(length_nm),
          temperature(temp_K), gen(std::random_device{}()),
          dist(0.0, 1.0) {}

    /**
     * Initialize water molecules in hexagonal lattice
     */
    void initialize_lattice(int n_layers = 3) {
        water.clear();

        double spacing = 0.28;  // nm, water-water distance

        int n_along = static_cast<int>(cavity_length / spacing);
        int n_around = static_cast<int>(2 * M_PI * cavity_radius / spacing);

        for (int i = 0; i < n_along; ++i) {
            for (int j = 0; j < n_around; ++j) {
                for (int k = 0; k < n_layers; ++k) {
                    double z = i * spacing;
                    double theta = 2 * M_PI * j / n_around;
                    double r = cavity_radius - k * spacing * 0.5;

                    if (r < 0) continue;

                    WaterMolecule w;
                    w.x = r * cos(theta);
                    w.y = r * sin(theta);
                    w.z = z;

                    // Initial random dipole orientation
                    w.dip_x = 2.0 * dist(gen) - 1.0;
                    w.dip_y = 2.0 * dist(gen) - 1.0;
                    w.dip_z = 2.0 * dist(gen) - 1.0;

                    // Normalize
                    double norm = sqrt(w.dip_x*w.dip_x +
                                        w.dip_y*w.dip_y +
                                        w.dip_z*w.dip_z);
                    w.dip_x /= norm;
                    w.dip_y /= norm;
                    w.dip_z /= norm;

                    w.energy = 0.0;

                    water.push_back(w);
                }
            }
        }

        std::cout << "Initialized " << water.size()
                  << " water molecules in cavity\n";
    }

    /**
     * Compute dipole-dipole interaction energy
     */
    double dipole_energy(const WaterMolecule& w1,
                         const WaterMolecule& w2) const {
        double dx = w1.x - w2.x;
        double dy = w1.y - w2.y;
        double dz = w1.z - w2.z;
        double r2 = dx*dx + dy*dy + dz*dz;
        double r = sqrt(r2);
        if (r < 1e-3) return 0.0;  // same molecule

        // Dipole-dipole interaction
        double d1d2 = w1.dip_x*w2.dip_x + w1.dip_y*w2.dip_y + w1.dip_z*w2.dip_z;
        double d1r = w1.dip_x*dx + w1.dip_y*dy + w1.dip_z*dz;
        double d2r = w2.dip_x*dx + w2.dip_y*dy + w2.dip_z*dz;

        double energy = (d1d2 - 3.0 * d1r * d2r / r2) / (r2 * r);
        return energy;
    }

    /**
     * Monte Carlo step to minimize energy (ordering)
     */
    void monte_carlo_step(double beta) {
        int idx = static_cast<int>(dist(gen) * water.size());
        WaterMolecule& w = water[idx];

        // Save old state
        double old_energy = w.energy;
        double old_dip_x = w.dip_x;
        double old_dip_y = w.dip_y;
        double old_dip_z = w.dip_z;

        // Propose new orientation (random rotation)
        double theta = 2 * M_PI * dist(gen);
        double phi = acos(2 * dist(gen) - 1);
        w.dip_x = sin(phi) * cos(theta);
        w.dip_y = sin(phi) * sin(theta);
        w.dip_z = cos(phi);

        // Compute new energy (nearest neighbors)
        double new_energy = 0.0;
        for (size_t i = 0; i < water.size(); ++i) {
            if (i == idx) continue;
            double dist = sqrt(pow(w.x - water[i].x, 2) +
                                pow(w.y - water[i].y, 2) +
                                pow(w.z - water[i].z, 2));
            if (dist < 0.6) {  // nearest neighbors within ~0.6 nm
                new_energy += dipole_energy(w, water[i]);
            }
        }

        // Accept/reject (Metropolis)
        double deltaE = new_energy - old_energy;
        if (deltaE < 0 || exp(-beta * deltaE) > dist(gen)) {
            w.energy = new_energy;
            // accept
        } else {
            // reject, restore old orientation
            w.dip_x = old_dip_x;
            w.dip_y = old_dip_y;
            w.dip_z = old_dip_z;
        }
    }

    /**
     * Compute order parameter (average alignment)
     */
    double compute_order() const {
        double avg_dip_x = 0.0, avg_dip_y = 0.0, avg_dip_z = 0.0;
        for (const auto& w : water) {
            avg_dip_x += w.dip_x;
            avg_dip_y += w.dip_y;
            avg_dip_z += w.dip_z;
        }
        avg_dip_x /= water.size();
        avg_dip_y /= water.size();
        avg_dip_z /= water.size();

        return sqrt(avg_dip_x*avg_dip_x + avg_dip_y*avg_dip_y + avg_dip_z*avg_dip_z);
    }

    /**
     * Run simulation
     */
    void run_simulation(int steps) {
        double beta = 1.0 / (kB * temperature);

        std::cout << "Running Monte Carlo simulation...\n";
        for (int step = 0; step < steps; ++step) {
            monte_carlo_step(beta);

            if (step % (steps/10) == 0) {
                double order = compute_order();
                std::cout << "Step " << step << ": order = " << order << "\n";
            }
        }

        double final_order = compute_order();
        std::cout << "Final order parameter: " << final_order << "\n";
    }
};

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Microtubule inner cavity: radius 7.5 nm, length 25 μm
    OrderedWater water(7.5, 25000.0, 300.0);

    water.initialize_lattice(3);
    water.run_simulation(100000);

    return 0;
}
