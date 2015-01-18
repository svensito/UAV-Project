// Circle Number
Pi = 3.14159265359

// environmental data
rho = 1.2682    // density [kg/m³]

// speed of trimmed flight
V_a = 10    // airspeed [m/s]

//airframe
b = 1.4     // wingspan [m]
l = 0.980  // length [m]
c = 0.27    // mean chord [m]
S = 0.38     // wing surface [m²]

// airframe data
// m_ZAGI = 1.56    // weight [kg] -> ZAGI
// m_AERO = 13.5    // weight [kg] -> AEROSONDE
m = 1.5             // weight in [kg] -> FUNCUB

// J_x_ZAGI = 0.1147    //
// J_x_AERO = 0.8244    // f
// J_y_ZAGI = 0.05760   // Moment of Inertia y axis [kg*m²] -> ZAGI
// J_y_AERO = 1.135     // Moment of Inertia y axis [kg*m²] -> AEROSONDE
// J_z_ZAGI = 0.1712    // Moment of Inertia z axis [kg*m²] -> ZAGI
// J_z_AERO = 1.1759    // Moment of Inertia z axis [kg*m²] -> AEROSONDE
// J_xz_ZAGI = 0.0015    // Moment of Inertia xz plane [kg*m²] -> ZAGI
// J_xz_AERO = 0.1204    // Moment of Inertia xz plane [kg*m²] -> AEROSONDE

// estimation by log analysis and simulation tuning to fit log
J_x = 0.5847            // Moment of Inertia x axis [kg*m²] -> FUNCUB (estimated)
J_y = 0.2135            // Moment of Inertia y axis [kg*m²] -> FUNCUB (estimated)
J_z = 0.359             // Moment of Inertia z axis [kg*m²] -> FUNCUB (estimated)
J_xz = 0.015            // Moment of Inertia xz plane [kg*m²] -> FUNCUB (estimated)
Gamma = J_x*J_z-(J_xz^2)    // as per formula 3.13

// aerodynamic moments for longitudinal movement
// C_L_0_ZAGI = 0.09167     // Lift Coefficient
// C_D_0_ZAGI = 0.01631
// C_m_0_ZAGI = -0.02338
// C_L_alpha_ZAGI = 3.5016
// C_D_alpha_ZAGI = 0.2108
// C_m_alpha_ZAGI = -0.5675
// C_m_alpha_AERO = -0.38
// C_L_q_ZAGI = 2.8932
// C_D_q_ZAGI = 0
// C_m_q_ZAGI = -1.3990
// C_m_q_AERO = -3.6
// C_L_de_ZAGI = 0.2724     // due to blended wing prefix is positive with ZAGI
// C_L_de_AERO = -0.36      // due to tail config prefix is minus with AEROSONDE
// C_D_de_ZAGI = 0.3045
// C_m_de_ZAGI = -0.3254
// C_m_de_AERO = -0.5

C_m_q     = -2.4      // Moment along y axis due to q -> Damping derivative (Funcub with high damping)
C_m_alpha = -0.39     // Moment along y axis due to alpha -> long stability derivative
C_m_de    = -0.018    // Moment along y axis due to elevator deflection
// one step elevator output is less than 1/10 of a degree!!!
// therefore the value is so low

a_theta_1 = -(rho*V_a^2*c*S/(2*J_y))*C_m_q*c/(2*V_a)
a_theta_2 = -(rho*V_a^2*c*S/(2*J_y))*C_m_alpha
a_theta_3 = (rho*V_a^2*c*S/(2*J_y))*C_m_de

s=%s

// Transfer Function for eta -> q
AC_long_nom = a_theta_3 * s
AC_long_denom = s^2 + a_theta_1 * s + a_theta_2

// 'c' for continuous, 'd' for discrete
//h = syslin('d',nom_,denom_)

//evans(h,100)

//sys = tf2ss(nom_/denom_)

// Step Response tested with 200 deflection

// Good GAINS for longitudinal PID found:
// Kp = 7
// Ki = 6
// Kd = 5
