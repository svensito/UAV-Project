// environmental data
rho = 1.2682    // density [kg/m³]

// speed of trimmed flight
V_a = 25    // airspeed [m/s]

//airframe
b = 1.4     // wingspan [m]
c = 0.25    // mean chord [m]
S = b*c     // wing surface

// airframe data
J_x = 0.1147        // Moment of Inertia x axis [kg*m²] -> estimated out of Table E.1 and E.2
//J_y = 0.5760        // Moment of Inertia y axis [kg*m²] -> ZAGI approx weight of Fun Cub / Aerosonde same config
J_y = 0.2860
J_z = 0.2860        // Moment of Inertia z axis [kg*m²]
J_xz = 0.015        // Moment of Inertia xz plane [kg*m²]    
Gamma = J_x*J_z-(J_xz^2)    // as per formula 3.13

// aerodynamic moments for longitudinal movement
C_m_q = -3.6       // Moment along y axis due to q
C_m_alpha = -0.4    // Moment along y axis due to alpha
C_m_de = -0.003       // Moment along y axis due to elevator deflection
// one step elevator output is less than 1/10 of a degree!!!
// therefore the value is so low

a_theta_1 = -(rho*V_a^2*c*S/(2*J_y))*C_m_q*c/(2*V_a)
a_theta_2 = -(rho*V_a^2*c*S/(2*J_y))*C_m_alpha
a_theta_3 = (rho*V_a^2*c*S/(2*J_y))*C_m_de

s=%s

// Transfer Function for eta -> q
nom_ = a_theta_3 * s
denom_ = s^2 + a_theta_1 * s + a_theta_2

// 'c' for continuous, 'd' for discrete
h = syslin('d',nom_,denom_)

//evans(h,100)

sys = tf2ss(nom_/denom_)

// Step Response tested with 200 deflection

// Good GAINS for longitudinal PID found:
// Kp = 7
// Ki = 6
// Kd = 5
