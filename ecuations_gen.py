import sympy as sp

q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')

def dh_matrix(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

A1 = dh_matrix(q1, 0.400, 0.025, -sp.pi/2)
A2 = dh_matrix(q2, 0, 0.455, 0)
A3 = dh_matrix(q3 - sp.pi/2, 0, 0.035, -sp.pi/2)
A4 = dh_matrix(q4, 0.420, 0, sp.pi/2)
A5 = dh_matrix(q5, 0, 0, -sp.pi/2)
A6 = dh_matrix(q6, 0.080, 0, 0)

print("Calculando T0_6")
T0_3 = sp.simplify(A1 * A2 * A3)
T0_6 = sp.simplify(T0_3 * A4 * A5 * A6)

p = T0_6[:3, 3]

print("\n=== ECUACIONES DE POSICIÓN (Para Sección V-C y FK Node) ===")
print("px =", p[0])
print("py =", p[1])
print("pz =", p[2])

print("\nCalculando Jacobiano Jv... (puede tardar unos segundos)")
Jv = p.jacobian([q1, q2, q3, q4, q5, q6])

print("\n=== MATRIZ JACOBIANA (Para Sección VI-A e IK Node) ===")
for i in range(3):
    for j in range(6):
        print(f"Jv[{i},{j}] = {sp.simplify(Jv[i,j])}")
