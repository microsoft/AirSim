namespace math_common
{
template <typename T>
inline T rad2deg(const T radians)
{
    return (radians / M_PI) * 180.0;
}

template <typename T>
inline T deg2rad(const T degrees)
{
    return (degrees / 180.0) * M_PI;
}

template <typename T>
inline T wrap_to_pi(T radians)
{
    int m = (int)(radians / (2 * M_PI));
    radians = radians - m * 2 * M_PI;
    if (radians > M_PI)
        radians -= 2.0 * M_PI;
    else if (radians < -M_PI)
        radians += 2.0 * M_PI;
    return radians;
}

template <typename T>
inline void wrap_to_pi_inplace(T& a)
{
    a = wrap_to_pi(a);
}

template <class T>
inline T angular_dist(T from, T to)
{
    wrap_to_pi_inplace(from);
    wrap_to_pi_inplace(to);
    T d = to - from;
    if (d > M_PI)
        d -= 2. * M_PI;
    else if (d < -M_PI)
        d += 2. * M_PI;
    return d;
}
}