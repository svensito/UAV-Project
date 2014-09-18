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
c = 0.25    // mean chord [m]
S = b*c     // wing surface

// airframe data
J_x = 0.5847        // Moment of Inertia x axis [kg*m²] -> estimated out of Table E.1 and E.2
//J_y = 0.5760        // Moment of Inertia y axis [kg*m²] -> ZAGI approx weight of Fun Cub / Aerosonde same config
J_y = 0.2860
J_z = 0.2860        // Moment of Inertia z axis [kg*m²]
J_xz = 0.025        // Moment of Inertia xz plane [kg*m²]    
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


