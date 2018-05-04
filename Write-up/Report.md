
# MPC Project Report

In MPC project we are given the waypoints in the global co-ordinate system.

We can use these points as it is but that would make calculations a bit complex, therefore for the sake of simplicity changing the waypoints to car's co-ordinate system and proceeding with the project.

This also means that we can consider px = 0, py = 0, and psi = 0. Why would be x, y and psi 0?

They will be 0 because we are considering car at the origin.

The steer value and throttle is used in the latency equations to update the state vectors. After updating all the calculations are performed.

I have considered reference velocity as 70 for the project (although it runs perfectly at 80 also) and this reference velocity is mentioned in MPC.cpp file

### Update equations used
    x_(t+1) = x_t  + v_t*cos(ψt)∗dt
    //x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt

    y_(t+1) = y_t + v_t*sin(ψt)∗dt
    //y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

    ψ_(t+1) =  ψ_t+v_t(/Lf)*δ∗dt
    //psi_[t+1] = psi[t] - (v[t] * delta[t] * dt) / Lf 

    v_(t+1) = v_t + a_t ∗ dt
    //v_[t+1] = v[t] + a[t] * dt

    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

    epsi[t+1] = psi[t] - psides[t] - (v[t] * delta[t]  * dt)/ Lf

These are the same equations mentioned in the classroom.


I have taken N as 10 because if I think it would be computationally more intensive if I increase it.

Also if N is decreased then we do not get to know if the equations are working correctly or not as then N would become small.



I have taken dt as 0.1 and if I increase the dt to 0.2 then car tends to move along the sides rather than center. 

If I reduce the dt to 0.05, car starts to oscillate. The trajectory becomes far from the ideal one. It would cause motion sickness and thus taking 0.1 as dt.



I have made a polynomial that fits the waypoints. Also latency is also added. Latency is needed because there can be delay for two reasons :

1) There might be the case where it takes time to compute.
2) There is delay when the signal is send and when the signal is executed.

To account for these delays a latency of 100 milliseconds has been implemented. A delay more then that may not be good in actual scenerio.  The delay has been taken into account by using the vehicle model equations. I am not taking the initial position  what is returned by simulator but it would be after 100 milliseconds according to vehicle model.

## Latency Equations
    double Latency = 0.1;
    px = v * Latency;
    cte = cte + v * sin(epsi) * Latency;
    psi = -1 * (v * steer_value * Latency) / Lf;
    epsi = epsi - ((v * steer_value * Latency) / Lf);
    v = v + throttle_value * Latency;

