#ifndef KALMAN_CLOCK_H
#define KALMAN_CLOCK_H

#include <Arduino.h>

/*
    Filtro de Kalman para estimar el error de un RTC usando conteo de ticks.


    Convención usada:
        f(k) = t_rtc - t_GPS = skew[ticks]
        phi(k) = T_RTC - T_GPS = offset[ticks]

    donde:
        t_rtc = ticks contados en 1 PPS
        t_gps = 32768 (cantidad del GPS)
        T_GPS = ticks acumulados ideales según el PPS
        T_RTC = ticks acumulados medidos del RTC

    Entonces:
        phi > 0  => faltaron ticks => RTC lento
        phi < 0  => sobraron ticks => RTC rápido

    Estado:
        x(k) = [ phi ]
               [  f  ]
        
        w(k) = [ w(phi) ]
               [  w(f)  ]

    Modelo:
        phi(k+1) = phi(k) + f(k) * pps_steps + w(phi)
        f(k+1)   = f(k) + w(f)

        x(k+1) = F x(k) + w(k)

        


    Si se procesa un PPS por vez:
        pps_steps = 1

    Si el loop procesa varios PPS juntos:
        pps_steps = seq_delta

    Medición usada:
        z = offset_ticks

    Por eso:
        H = [1 0]
*/

struct Kalman_Filter_Config {
    // Ruido del proceso para phi.
    // Unidad: ticks^2 por paso PPS.
    double q_w_phi = 0.001;

    // Ruido del proceso para f.
    // Unidad: (ticks/PPS)^2 por paso PPS.
    double q_w_f = 0.000001;

    // Ruido de medición de phi.
    // Unidad: ticks^2.
    // 0.25 equivale a sigma = 0.5 tick.
    double r_phi_ticks2 = 0.25;

    // Incertidumbre inicial del offset.
    // Unidad: ticks^2.
    double p_phi0_ticks2 = 100.0;

    // Incertidumbre inicial del skew.
    // Unidad: (ticks/PPS)^2.
    double p_f0_ticks_per_pps2 = 1.0;
};

struct Kalman_Filter_State {
    double phi_ticks = 0.0;
    double f_ticks_per_pps = 0.0;

    // Matriz de covarianza P:
    // [ p00 p01 ]
    // [ p10 p11 ]
    double p00 = 0.0;
    double p01 = 0.0;
    double p10 = 0.0;
    double p11 = 0.0;

    // Innovación: y = z - H*x_pred
    double innovation_ticks = 0.0;

    // Covarianza de la innovación: S = H*P*H^T + R
    double innovation_covariance = 0.0;

    // Ganancia de Kalman:
    // K = [ k_phi ]
    //     [ k_f   ]
    double k_phi = 0.0;
    double k_f = 0.0;
};

class Kalman_Filter {
public:
    explicit Kalman_Filter(const Kalman_Filter_Config& config = Kalman_Filter_Config());

    void reset(
        double initial_phi_ticks = 0.0,
        double initial_f_ticks_per_pps = 0.0
    );

    void predict(double pps_steps);

    void updatePhi(double measured_phi_ticks);

    void step(
        double measured_phi_ticks,
        double pps_steps
    );

   Kalman_Filter_State getState() const;

    double getPhiTicks() const;
    double getPhiMicroseconds() const;

    double getFTicksPerPps() const;
    double getFPpmEquivalent() const;

private:
    Kalman_Filter_Config cfg;

    double phi;
    double f;

    double p00;
    double p01;
    double p10;
    double p11;

    double lastInnovation;
    double lastS;
    double lastKPhi;
    double lastKF;
};

#endif