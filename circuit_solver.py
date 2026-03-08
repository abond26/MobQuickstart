#!/usr/bin/env python3
"""
Circuit Analysis: Three parallel branches with resistors and batteries
"""

# Given values
R1 = 89.8  # ohms
R2 = 20.7  # ohms
R3 = 70.0  # ohms
E1 = 40.0  # volts
E2 = 368.0  # volts

# Using Kirchhoff's laws:
# For each branch, the voltage difference V_top - V_bottom must be the same
# Left branch: V_top - V_bottom = I1*R1 - E1 (if I1 flows down, drop across R1 is I1*R1, then -E1 going through battery)
# Actually, let's be more careful about signs

# If we traverse left branch from top to bottom:
# - Through R1: if I1 flows down, voltage drop = I1*R1
# - Through E1: battery positive terminal up, so going from + to - gives -E1
# So: V_top - V_bottom = I1*R1 - E1

# Similarly for middle: V_top - V_bottom = I2*R2 - E2
# For right: V_top - V_bottom = I3*R3

# All three must be equal, so:
# I1*R1 - E1 = I2*R2 - E2 = I3*R3

# Also, KCL at top node: I1 + I2 + I3 = 0

# Let V = V_top - V_bottom
# Then: V = I1*R1 - E1  =>  I1 = (V + E1)/R1
#       V = I2*R2 - E2  =>  I2 = (V + E2)/R2
#       V = I3*R3       =>  I3 = V/R3

# KCL: I1 + I2 + I3 = 0
# (V + E1)/R1 + (V + E2)/R2 + V/R3 = 0
# V*(1/R1 + 1/R2 + 1/R3) + E1/R1 + E2/R2 = 0

# Calculate equivalent resistance
R_eq = 1 / (1/R1 + 1/R2 + 1/R3)

# Calculate V
V = -(E1/R1 + E2/R2) / (1/R1 + 1/R2 + 1/R3)

# Calculate currents
I1 = (V + E1) / R1
I2 = (V + E2) / R2
I3 = V / R3

# Calculate potential differences
dV_R1 = abs(I1 * R1)
dV_R2 = abs(I2 * R2)
dV_R3 = abs(I3 * R3)

print("Circuit Analysis Results:")
print(f"Voltage difference V_top - V_bottom: {V:.4f} V")
print()
print("Part (a) - Currents:")
print(f"I₁ = {I1:.4f} A")
print(f"I₂ = {I2:.4f} A")
print(f"I₃ = {I3:.4f} A")
print()
print("Part (b) - Potential Differences:")
print(f"|ΔV_R₁| = {dV_R1:.4f} V")
print(f"|ΔV_R₂| = {dV_R2:.4f} V")
print(f"|ΔV_R₃| = {dV_R3:.4f} V")
print()
print("Verification (KCL):")
print(f"I₁ + I₂ + I₃ = {I1 + I2 + I3:.6f} A (should be ~0)")
