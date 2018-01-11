#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Fiddle with the timestep length and duration, as necessary
size_t N = 10;
double dt = .1;

////////////////////////////////////////////////
// Reference vars brought in from MPC quizzes //
////////////////////////////////////////////////

// Both the reference cross track and orientation errors are 0.
double ref_cte = 0;
double ref_epsi = 0;

// The reference velocity; in MPH.
double ref_v = 16;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lives easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t accel_start = delta_start + N - 1;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    /**
     * @param fg  a vector of the cost constraints
     * @param vars  a vector of variable values (state & actuators)
     */
    void operator()(ADvector &fg, const ADvector &vars) {
        // The cost is stored is the first element of `fg`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (int t = 0; t < N; t++) {
            // Emphasize keeping CTE and error-psi low!
            fg[0] += 2000 * CppAD::pow(vars[cte_start + t], 2);
            fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
            fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
        }

        // Minimize the use of actuators.
        for (int t = 0; t < N - 1; t++) {
            fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
            fg[0] += 5 * CppAD::pow(vars[accel_start + t], 2);
        }

        // Minimize the value gap between sequential actuations.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += 10 * CppAD::pow(vars[accel_start + t + 1] - vars[accel_start + t], 2);
        }

        ///////////////////////
        // Setup Constraints //
        ///////////////////////

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints, over time
        for (int t = 1; t < N; ++t) {
            // The state at time t+1 .
            AD<double> x1 = vars[x_start + t];
            AD<double> y1 = vars[y_start + t];
            AD<double> psi1 = vars[psi_start + t];
            AD<double> v1 = vars[v_start + t];
            AD<double> cte1 = vars[cte_start + t];
            AD<double> epsi1 = vars[epsi_start + t];

            // The state at time t.
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> y0 = vars[y_start + t - 1];
            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> v0 = vars[v_start + t - 1];
            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> epsi0 = vars[epsi_start + t - 1];

            // Only consider the actuation at time t.
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[accel_start + t - 1];

            AD<double> x0_squared = x0 * x0;
            AD<double> x0_cubed = x0_squared * x0;
            AD<double> f0 =  coeffs[0]     + coeffs[1] * x0 + coeffs[2] * x0_squared   + coeffs[3] * x0_cubed;
            AD<double> psides0 = CppAD::atan(coeffs[1]      + 2 * coeffs[2] * x0       + 3 * coeffs[3] * x0_squared);


            // The idea here is to constraint this value to be 0.

            // The equations for the model, from lectures:
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
            fg[1 + x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + t]   = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + v_start + t]     = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // number of independent variables
    size_t n_vars = N * 6 + (N - 1) * 2; // N timesteps == N - 1 actuations
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0.;
    }

    // Set the initial variable values

    const double x     = state[0];
    const double y     = state[1];
    const double psi   = state[2];
    const double v     = state[3];
    const double cte   = state[4];
    const double epsi  = state[5];

    vars[x_start]     = x;
    vars[y_start]     = y;
    vars[psi_start]   = psi;
    vars[v_start]     = v;
    vars[cte_start]   = cte;
    vars[epsi_start]  = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = delta_start; i < accel_start; i++) {
        // NOTE: Still not sure if *Lf should be included or not...
        vars_lowerbound[i] = -0.436332 * Lf;
        vars_upperbound[i] = 0.436332 * Lf;
    }

    // Acceleration/decceleration upper and lower limits.
    for (int i = accel_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // Maximum time limit to calc, in seconds.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    if (!ok) {
        std::cout << "WARN: Solution.statue returned to be NOT OK!" << std::endl;
    }
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    vector<double> result;
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[accel_start]);

    for (int i = 0; i < N-1; ++i) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }

    return result;
}
