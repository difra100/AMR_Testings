def RK4(f, x0, u, dt):
    k1 = f(x0, u)
    k2 = f(x0 + k1 * dt / 2.0, u)
    k3 = f(x0 + k2 * dt / 2.0, u)
    k4 = f(x0 + k3 * dt, u)
    yf = x0 + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return yf
