from scipy.optimize import minimize
import numpy as np

def constrained_taylor_fit(x, y, degree=4, x0=0.0):
    """
    Fit a polynomial with Taylor structure (centered at x0), constrained to f(x) ≥ 0 and ≤ 1.
    Returns coefficients and callable function.
    """
    x_shifted = x - x0
    powers = np.vstack([x_shifted**i for i in range(degree + 1)]).T  # shape: (N, degree+1)

    def model(c):
        return powers @ c

    def loss(c):
        return np.sum((model(c) - y)**2)

    # Constraint: f(x) ≥ 0
    def constraint_lower(c):
        return model(c)  # f(x) ≥ 0 → model(c) ≥ 0

    # Constraint: f(x) ≤ 1
    def constraint_upper(c):
        return 1.0 - model(c)

    cons = [
        {'type': 'ineq', 'fun': constraint_lower},
        {'type': 'ineq', 'fun': constraint_upper},
    ]

    # Initial guess from polyfit
    c0 = np.polyfit(x_shifted, y, degree)[::-1]

    result = minimize(loss, c0, constraints=cons)
    coeffs = result.x
    f = lambda x_input: np.polyval(coeffs[::-1], np.array(x_input) - x0)

    # Print formula
    terms = []
    for i, c in enumerate(coeffs):
        if abs(c) < 1e-6:
            continue
        term = f"{c:.4f}"
        if i == 1:
            term += "*x"
        elif i > 1:
            term += f"*x**{i}"
        terms.append(term)
    print("Constrained Polynomial (Taylor-like):")
    print("f(x) = " + " + ".join(terms))

    return coeffs, f
