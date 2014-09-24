// Lateral Transfer Function (Roll)

// environmental data
rho = 1.2682    // density [kg/m³]

// gravity
g = 9.81        // [m/s²]

// circle number Pi
Pi = 3.14159265359

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

// aerodynamic moments for latitudinal movement
// Inertial Data
Gamma = J_x*J_z-J_xz^2
Gamma_1 = (J_xz*(J_x-J_y+J_z)) / Gamma
Gamma_2 = (J_z*(J_z-J_y)+J_xz^2) / Gamma
Gamma_3 = J_z / Gamma
Gamma_4 = J_xz / Gamma
Gamma_5 = (J_z - J_x)/J_y
Gamma_6 = J_xz / J_y
Gamma_7 = ((J_x - J_y)*J_x+J_xz^2)/Gamma
Gamma_8 = J_x/Gamma

C_l_p = -0.3209
C_n_p = -0.01297
C_l_da = 0.00282
C_n_da = -0.00528

C_p_p = Gamma_3*C_l_p + Gamma_4*C_n_p
C_p_da = Gamma_3*C_l_da + Gamma_4*C_n_da


// one step aileron output is less than 1/10 of a degree!!!
// therefore the value is so low

a_phi_1 = -(rho*V_a^2*b*S)*C_p_p*b/(2*V_a)
a_phi_2 = (rho*V_a^2*b*S*C_p_da)


s=%s

// Transfer Function for zeta -> q
nom_ = a_phi_2
denom_ = s + a_phi_1

// 'c' for continuous, 'd' for discrete
h = syslin('d',nom_,denom_)

//evans(h,100)

sys = tf2ss(nom_/denom_)

// FOUND PI VALUES
// Kp = 15
// Ki = 5

// Transfer Function phi -> chi


