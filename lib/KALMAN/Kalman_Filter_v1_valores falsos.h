#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

/*
    Traducción directa del apunte de Kalman.

    Estado:

        x = [ phi ]
            [  f  ]

    Modelo:

        x(k+1) = F x(k) + w(k)

        phi(k+1) = phi(k) + f(k) + w_phi
        f(k+1)   = f(k) + w_f

        F = [1 1]
            [0 1]

    Ruido del proceso:

        w ~ N(0, Q)

        Q = E[w w^T]

        Q = [ sigma_phi^2      0      ]
            [      0       sigma_f^2 ]

    Medición:

        z = H x + v

        v ~ N(0, R)

        R = E[v v^T]

    Predicción:

        x_pred(k) = F x_pred(k-1)
        P_pred(k) = F P_pred(k-1) F^T + Q

    Error de Estimación:

        e  = x - x_pred

    Innovación:

        y = z - H x_pred
        S = H P_pred H^T + R

    Actualización:

        K = P_pred H^T S^-1
        x = x_pred + K y
        P = (I - K H) P_pred
*/

struct Kalman_Config {
    /*
        Q = [ q_phi  0   ]
            [  0    q_f ]

        q_phi = sigma_phi^2
        q_f   = sigma_f^2

        Representan el ruido del modelo.
    */
    double q_phi = 0.001;
    double q_f   = 0.000001;

    /*
        R para el caso en que se mide phi:

            z = phi
            H = [1 0]
            R = r_phi
    */
    double r_phi = 0.25;

    /*
        R para el caso en que se mide f:

            z = f
            H = [0 1]
            R = r_f
    */
    double r_f = 0.25;

    /*
        P inicial:

            P = E[e e^T]

        donde:

            e = x_real - x_pred

        P representa la incertidumbre de la estimación.
    */
    double p_phi_0 = 100.0;
    double p_f_0   = 1.0;
};

struct Kalman_State {
    /*
        x = [ phi ]
            [  f  ]
    */
    double phi = 0.0;
    double f   = 0.0;

    /*
        P = [ p00 p01 ]
            [ p10 p11 ]
    */
    double p00 = 0.0;
    double p01 = 0.0;
    double p10 = 0.0;
    double p11 = 0.0;

    /*
        Innovación:

            y = z - H x_pred
    */
    double y0 = 0.0;
    double y1 = 0.0;

    /*
        Ganancia de Kalman.

        Si se mide una sola variable, K es columna:

            K = [ k0 ]
                [ k1 ]

        Si se miden phi y f, K es matriz 2x2.
    */
    double k00 = 0.0;
    double k01 = 0.0;
    double k10 = 0.0;
    double k11 = 0.0;
};

class Kalman_Filter {
public:
    explicit Kalman_Filter(const Kalman_Config& config = Kalman_Config());

    void reset(double initial_phi = 0.0, double initial_f = 0.0);

    /*
        Predicción:

            x_pred(k) = F x_pred(k-1)
            P_pred(k) = F P_pred(k-1) F^T + Q

        con:

            F = [1 1]
                [0 1]
    */
    void predict();

    /*
        Medición de phi:

            z = phi
            H = [1 0]
            R = r_phi
    */
    void updatePhi(double z_phi);

    /*
        Medición de f:

            z = f
            H = [0 1]
            R = r_f
    */
    void updateF(double z_f);

    /*
        Medición de phi y f:

            z = [ phi ]
                [  f  ]

            H = [1 0]
                [0 1]

            R = [ r_phi   0  ]
                [   0    r_f ]
    */
    void updatePhiAndF(double z_phi, double z_f);

    Kalman_State getState() const;

private:
    Kalman_Config cfg;

    /*
        Estado estimado:

            x_hat = [ phi ]
                    [  f  ]
    */
    double phi;
    double f;

    /*
        Matriz de covarianza del error de estimación:

            P = [ p00 p01 ]
                [ p10 p11 ]
    */
    double p00;
    double p01;
    double p10;
    double p11;

    /*
        Últimos valores guardados para mirar qué hizo el filtro.
    */
    double y0;
    double y1;

    double k00;
    double k01;
    double k10;
    double k11;
};

#endif