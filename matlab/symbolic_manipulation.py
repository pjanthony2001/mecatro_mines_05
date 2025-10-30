from sympy import symbols, Eq, solve, collect, factor_terms

A = symbols('A0:8')
u_dot, theta, theta_dot_dot, i_plus = symbols('u_dot theta theta_dot_dot i_plus')
u, theta_dot, rho = symbols('u theta_dot rho')


eq1 = Eq(A[0] * u_dot + A[1] * theta_dot_dot + A[2] * i_plus, 0)
eq2 = Eq(A[3] * u_dot + A[4] * theta_dot_dot + A[5] * theta + A[6] * i_plus, 0)

# solve the system for theta and i_plus
solution = solve((eq1, eq2), (theta, i_plus))

B_ = [0, 0, 0, 0, 0, 0, 0]
B_[0] = solution[theta].as_poly(u_dot).all_coeffs()[0]
B_[1] = solution[theta].as_poly(theta_dot_dot).all_coeffs()[0]


# solve the system for theta_dot_dot and u_dot
solution = solve((eq1, eq2), (theta_dot_dot, u_dot))

B_[3] = solution[theta_dot_dot].as_poly(theta).all_coeffs()[0]
B_[4] = solution[theta_dot_dot].as_poly(i_plus).all_coeffs()[0]

B = symbols('B0:8')



print(B_)

mu, nu, nu_dot, p, psi = symbols('mu nu nu_dot p psi')
k, R, U_l, U_r, Omega_l, Omega_r, U_plus = symbols('k R U_l U_r Omega_l Omega_r U_plus')



eq2_1 = Eq(theta_dot_dot, B[3] * theta + B[4] * i_plus)
           
           
eq3 = Eq(mu, B[0] * p + B[1] * theta)
eq4 = Eq(nu, B[0] * u + B[1] * theta_dot)

eq4_1 = Eq(u, solve(eq4, u)[0]) # u as a fn of nu and theta_dot


eq5 = Eq(i_plus, (U_l + U_r - k * (Omega_l + Omega_r)) / R)
eq6 = Eq((Omega_l + Omega_r), 2 * (u / rho - theta_dot))


eq6_1 = Eq(U_plus, U_l + U_r)

eq7 = eq5.subs([(eq6.lhs, eq6.rhs), (eq4_1.lhs, eq4_1.rhs), (eq6_1.rhs, eq6_1.lhs)])

C_ = [0, 0, 0, 0, 0, 0]
C_[0] = eq7.rhs.as_poly(nu).all_coeffs()[0]
C_[1] = eq7.rhs.as_poly(theta_dot).all_coeffs()[0]
C_[2] = eq7.rhs.as_poly(U_plus).all_coeffs()[0]

print(C_)

C = symbols('C0:8')

eq8 = Eq(i_plus, C[0] * nu + C[1] * theta_dot + C[2] * U_plus)

print(eq2_1.subs(eq8.lhs, eq8.rhs))


