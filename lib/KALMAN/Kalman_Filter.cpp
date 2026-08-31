#include "Kalman_Filter.h"

Kalman_Filter::Kalman_Filter(const Kalman_Config& config)
    : cfg(config) {
    reset();
}

void Kalman_Filter::reset(double initial_phi, double initial_f) {
    /*
        Estado inicial:

            x = [ phi ]
                [  f  ]
    */
    phi = initial_phi;
    f   = initial_f;

    /*
        P inicial:

            P = [ p_phi_0    0    ]
                [    0     p_f_0  ]

        Se arranca con covarianza cruzada cero.
    */
    p00 = cfg.p_phi_0;
    p01 = 0.0;
    p10 = 0.0;
    p11 = cfg.p_f_0;

    y0 = 0.0;
    y1 = 0.0;

    k00 = 0.0;
    k01 = 0.0;
    k10 = 0.0;
    k11 = 0.0;
}

void Kalman_Filter::predict() {
    /*
        Modelo:

            x_pred(k) = F x_pred(k-1)

        donde:

            F = [1 1]
                [0 1]

        Entonces:

            phi_pred = phi + f
            f_pred   = f
    */

    const double phi_pred = phi + f;
    const double f_pred   = f;

    phi = phi_pred;
    f   = f_pred;

    /*
        Covarianza:

            P_pred(k) = F P_pred(k-1) F^T + Q

        con:

            P = [ p00 p01 ]
                [ p10 p11 ]

            F = [1 1]
                [0 1]

            Q = [ q_phi  0   ]
                [  0    q_f ]

        Desarrollo:

            p00_pred = p00 + p01 + p10 + p11 + q_phi
            p01_pred = p01 + p11
            p10_pred = p10 + p11
            p11_pred = p11 + q_f
    */

    const double p00_pred = p00 + p01 + p10 + p11 + cfg.q_phi;
    const double p01_pred = p01 + p11;
    const double p10_pred = p10 + p11;
    const double p11_pred = p11 + cfg.q_f;

    p00 = p00_pred;
    p01 = p01_pred;
    p10 = p10_pred;
    p11 = p11_pred;
}

void Kalman_Filter::updatePhi(double z_phi) {
    /*
        Caso:

            z = phi

            H = [1 0]

            R = r_phi

        Predicción de la medición:

            z_pred = H x_pred
            z_pred = phi

        Innovación:

            y = z - z_pred
    */

    const double z_pred = phi;
    const double y = z_phi - z_pred;

    /*
        S = H P H^T + R

        Como H = [1 0]:

            S = p00 + r_phi
    */
    const double S = p00 + cfg.r_phi;

    /*
        K = P H^T S^-1

        Como H = [1 0]:

            K = [ p00 / S ]
                [ p10 / S ]
    */
    const double k0 = p00 / S;
    const double k1 = p10 / S;

    /*
        Actualización del estado:

            x = x_pred + K y
    */
    phi = phi + k0 * y;
    f   = f   + k1 * y;

    /*
        Actualización de P:

            P = (I - K H) P_pred

        con:

            K = [ k0 ]
                [ k1 ]

            H = [1 0]

            I - K H = [1-k0    0]
                      [ -k1    1]
    */

    const double old_p00 = p00;
    const double old_p01 = p01;
    const double old_p10 = p10;
    const double old_p11 = p11;

    p00 = (1.0 - k0) * old_p00;
    p01 = (1.0 - k0) * old_p01;
    p10 = old_p10 - k1 * old_p00;
    p11 = old_p11 - k1 * old_p01;

    y0 = y;
    y1 = 0.0;

    k00 = k0;
    k01 = 0.0;
    k10 = k1;
    k11 = 0.0;
}

void Kalman_Filter::updateF(double z_f) {
    /*
        Caso:

            z = f

            H = [0 1]

            R = r_f

        Predicción de la medición:

            z_pred = H x_pred
            z_pred = f

        Innovación:

            y = z - z_pred
    */

    const double z_pred = f;
    const double y = z_f - z_pred;

    /*
        S = H P H^T + R

        Como H = [0 1]:

            S = p11 + r_f
    */
    const double S = p11 + cfg.r_f;

    /*
        K = P H^T S^-1

        Como H = [0 1]:

            K = [ p01 / S ]
                [ p11 / S ]
    */
    const double k0 = p01 / S;
    const double k1 = p11 / S;

    /*
        Actualización del estado:

            x = x_pred + K y
    */
    phi = phi + k0 * y;
    f   = f   + k1 * y;

    /*
        P = (I - K H) P

        con:

            K = [ k0 ]
                [ k1 ]

            H = [0 1]

            I - K H = [1   -k0]
                      [0  1-k1]
    */

    const double old_p00 = p00;
    const double old_p01 = p01;
    const double old_p10 = p10;
    const double old_p11 = p11;

    p00 = old_p00 - k0 * old_p10;
    p01 = old_p01 - k0 * old_p11;
    p10 = (1.0 - k1) * old_p10;
    p11 = (1.0 - k1) * old_p11;

    y0 = y;
    y1 = 0.0;

    k00 = k0;
    k01 = 0.0;
    k10 = k1;
    k11 = 0.0;
}

void Kalman_Filter::updatePhiAndF(double z_phi, double z_f) {
    /*
        Caso:

            z = [ phi ]
                [  f  ]

            H = [1 0]
                [0 1]

        Como H es identidad:

            z_pred = x_pred

        Innovación:

            y = z - z_pred
    */

    const double y_phi = z_phi - phi;
    const double y_f   = z_f   - f;

    /*
        S = H P H^T + R

        Como H = I:

            S = P + R

        con:

            R = [ r_phi   0  ]
                [   0    r_f ]

        Entonces:

            S = [ p00 + r_phi      p01     ]
                [    p10       p11 + r_f  ]
    */

    const double s00 = p00 + cfg.r_phi;
    const double s01 = p01;
    const double s10 = p10;
    const double s11 = p11 + cfg.r_f;

    /*
        S^-1 para matriz 2x2:

            S^-1 = 1/det * [ s11  -s01 ]
                          [ -s10   s00 ]

        donde:

            det = s00*s11 - s01*s10
    */

    const double det = s00 * s11 - s01 * s10;

    if (det == 0.0) {
        return;
    }

    const double inv_s00 =  s11 / det;
    const double inv_s01 = -s01 / det;
    const double inv_s10 = -s10 / det;
    const double inv_s11 =  s00 / det;

    /*
        K = P H^T S^-1

        Como H = I:

            K = P S^-1
    */

    const double k_00 = p00 * inv_s00 + p01 * inv_s10;
    const double k_01 = p00 * inv_s01 + p01 * inv_s11;
    const double k_10 = p10 * inv_s00 + p11 * inv_s10;
    const double k_11 = p10 * inv_s01 + p11 * inv_s11;

    /*
        x = x_pred + K y
    */

    phi = phi + k_00 * y_phi + k_01 * y_f;
    f   = f   + k_10 * y_phi + k_11 * y_f;

    /*
        P = (I - K H) P

        Como H = I:

            P = (I - K) P
    */

    const double old_p00 = p00;
    const double old_p01 = p01;
    const double old_p10 = p10;
    const double old_p11 = p11;

    const double a00 = 1.0 - k_00;
    const double a01 =     - k_01;
    const double a10 =     - k_10;
    const double a11 = 1.0 - k_11;

    p00 = a00 * old_p00 + a01 * old_p10;
    p01 = a00 * old_p01 + a01 * old_p11;
    p10 = a10 * old_p00 + a11 * old_p10;
    p11 = a10 * old_p01 + a11 * old_p11;

    y0 = y_phi;
    y1 = y_f;

    k00 = k_00;
    k01 = k_01;
    k10 = k_10;
    k11 = k_11;
}

Kalman_State Kalman_Filter::getState() const {
    Kalman_State state;

    state.phi = phi;
    state.f   = f;

    state.p00 = p00;
    state.p01 = p01;
    state.p10 = p10;
    state.p11 = p11;

    state.y0 = y0;
    state.y1 = y1;

    state.k00 = k00;
    state.k01 = k01;
    state.k10 = k10;
    state.k11 = k11;

    return state;
}