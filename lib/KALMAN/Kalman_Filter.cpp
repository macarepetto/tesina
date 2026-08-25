#include "Kalman_Filter.h"

static constexpr double TICKS_PER_PPS_IDEAL = 32768.0;

Kalman_Filter::Kalman_Filter(const Kalman_Filter_Config& config)
    : cfg(config) {
    reset();
}

void Kalman_Filter::reset(
    double initial_phi_ticks,
    double initial_f_ticks_per_pps
) {
    phi = initial_phi_ticks;
    f = initial_f_ticks_per_pps;

    p00 = cfg.p_phi0_ticks2;
    p01 = 0.0;
    p10 = 0.0;
    p11 = cfg.p_f0_ticks_per_pps2;

    lastInnovation = 0.0;
    lastS = 0.0;
    lastKPhi = 0.0;
    lastKF = 0.0;
}

void Kalman_Filter::predict(double pps_steps) {
    if (pps_steps <= 0.0) {
        pps_steps = 1.0;
    }

    /*
       Estado:
           x = [ phi ]
               [  f  ]

       Modelo:
           phi(k+1) = phi(k) + f(k) * pps_steps
           f(k+1)   = f(k)

       Matriz:
           F = [1  pps_steps]
               [0      1    ]
    */
    phi = phi + f * pps_steps;

    /*
       P_pred = F * P * F^T + Q

       Para:
           F = [1 n]
               [0 1]

       queda:
           p00' = p00 + n*p10 + n*p01 + n^2*p11 + q_phi
           p01' = p01 + n*p11
           p10' = p10 + n*p11
           p11' = p11 + q_f

       Como q_phi y q_f están definidos por paso PPS,
       si se procesan varios PPS juntos se multiplican por pps_steps.
    */
    const double n = pps_steps;

    const double q_phi_total = cfg.q_w_phi * n;
    const double q_f_total = cfg.q_w_f * n;

    const double new_p00 =
        p00 + n * p10 + n * p01 + n * n * p11 + q_phi_total;

    const double new_p01 =
        p01 + n * p11;

    const double new_p10 =
        p10 + n * p11;

    const double new_p11 =
        p11 + q_f_total;

    p00 = new_p00;
    p01 = new_p01;
    p10 = new_p10;
    p11 = new_p11;
}

void Kalman_Filter::updatePhi(double measured_phi_ticks) {
    /*
       Medición:
           z = phi + ruido

       H = [1 0]

       Predicción de medición:
           z_pred = H*x = phi

       Innovación:
           y = z - z_pred

       Covarianza de innovación:
           S = H*P*H^T + R = p00 + R

       Ganancia:
           K = P*H^T*S^-1
           K = [p00/S]
               [p10/S]
    */
    const double z_pred = phi;
    const double y = measured_phi_ticks - z_pred;
    const double S = p00 + cfg.r_phi_ticks2;

    const double k0 = p00 / S;
    const double k1 = p10 / S;

    // x = x_pred + K*y
    phi = phi + k0 * y;
    f = f + k1 * y;

    /*
       P = (I - K*H) * P

       Con H = [1 0]:

           I - K*H = [1-k0   0]
                     [ -k1   1]
    */
    const double old_p00 = p00;
    const double old_p01 = p01;
    const double old_p10 = p10;
    const double old_p11 = p11;

    p00 = (1.0 - k0) * old_p00;
    p01 = (1.0 - k0) * old_p01;
    p10 = old_p10 - k1 * old_p00;
    p11 = old_p11 - k1 * old_p01;

    lastInnovation = y;
    lastS = S;
    lastKPhi = k0;
    lastKF = k1;
}

void Kalman_Filter::step(
    double measured_phi_ticks,
    double pps_steps
) {
    predict(pps_steps);
    updatePhi(measured_phi_ticks);
}

Kalman_Filter_State Kalman_Filter::getState() const {
    Kalman_Filter_State state;

    state.phi_ticks = phi;
    state.f_ticks_per_pps = f;

    state.p00 = p00;
    state.p01 = p01;
    state.p10 = p10;
    state.p11 = p11;

    state.innovation_ticks = lastInnovation;
    state.innovation_covariance = lastS;

    state.k_phi = lastKPhi;
    state.k_f = lastKF;

    return state;
}

double Kalman_Filter::getPhiTicks() const {
    return phi;
}

double Kalman_Filter::getPhiMicroseconds() const {
    /*
       1 tick = 1 / 32768 s

       Se convierte phi desde ticks acumulados a microsegundos
       solo para que sea más legible en el JSON o en los gráficos.
    */
    return phi * 1000000.0 / TICKS_PER_PPS_IDEAL;
}

double Kalman_Filter::getFTicksPerPps() const {
    return f;
}

double Kalman_Filter::getFPpmEquivalent() const {
    /*
       f está en ticks/PPS.

       Como el ideal es:
           32768 ticks/PPS

       entonces:
           ppm = f / 32768 * 1e6

       Signo según la convención del código:
           ppm equivalente positivo => faltan ticks => RTC lento
           ppm equivalente negativo => sobran ticks => RTC rápido
    */
    return f * 1000000.0 / TICKS_PER_PPS_IDEAL;
}