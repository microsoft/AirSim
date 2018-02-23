/*
Adopted from SunCalc by Vladimir Agafonkin
https://github.com/mourner/suncalc
*/

#ifndef airsim_core_EarthCelestial_hpp
#define airsim_core_EarthCelestial_hpp


#include "common/Common.hpp"
#include "EarthUtils.hpp"
#include <chrono>
#include <ctime>

namespace msr { namespace airlib {


class EarthCelestial {
public:

    struct CelestialGlobalCoord
    {
        double declination;
        double rightAscension;
        double distance = Utils::nan<double>();
        double parallacticAngle = Utils::nan<double>();
    };

    struct CelestialLocalCoord
    {
        double azimuth;
        double altitude;
        double distance = Utils::nan<double>();
        double parallacticAngle = Utils::nan<double>();
    };

    struct CelestialPhase
    {
        double fraction;
        double phase;
        double angle;
    };


public:
    static CelestialLocalCoord getSunCoordinates(uint64_t date, double lat, double lng)
    {
        double lw  = Utils::degreesToRadians(-lng);
        double phi = Utils::degreesToRadians(lat);
        double d  = toDays(date);

        CelestialGlobalCoord c = getGlobalSunCoords(d);
        double H = siderealTime(d, lw) - c.rightAscension;

        CelestialLocalCoord coord;
        coord.azimuth = Utils::radiansToDegrees( azimuth(H, phi, c.declination) ) + 180.0;
        coord.altitude = Utils::radiansToDegrees( altitude(H, phi, c.declination) );

        return coord;
    }


    static CelestialLocalCoord getMoonCoordinates(uint64_t date, double lat, double lng)
    {

        double lw  = Utils::degreesToRadians(-lng);
        double phi = Utils::degreesToRadians(lat);
        double d = toDays(date);

        CelestialGlobalCoord c = getGlobalMoonCoords(d);
        double H = siderealTime(d, lw) - c.rightAscension;

        // formula 14.1 of "Astronomical Algorithms" 2nd edition by Jean Meeus (Willmann-Bell, Richmond) 1998.
        double pa = std::atan2(std::sin(H), std::tan(phi) * std::cos(c.declination) - std::sin(c.declination) * std::cos(H));

        double h = altitude(H, phi, c.declination);
        h = h + astroRefraction(h); // altitude correction for refraction

        CelestialLocalCoord coord;
        coord.azimuth = Utils::radiansToDegrees( azimuth(H, phi, c.declination) );
        coord.altitude = Utils::radiansToDegrees(h);
        coord.distance = c.distance;
        coord.parallacticAngle = Utils::radiansToDegrees(pa);
        return coord;
    };


    // calculations for illumination parameters of the moon,
    // based on http://idlastro.gsfc.nasa.gov/ftp/pro/astro/mphase.pro formulas and
    // Chapter 48 of "Astronomical Algorithms" 2nd edition by Jean Meeus (Willmann-Bell, Richmond) 1998.
    static CelestialPhase getMoonPhase(uint64_t date)
    {
        double d = toDays(date);
        CelestialGlobalCoord s = getGlobalSunCoords(d);
        CelestialGlobalCoord m = getGlobalMoonCoords(d);

        double sdist = EarthUtils::DistanceFromSun / 1000; // distance from Earth to Sun in km

        double phi = std::acos(std::sin(s.declination) * std::sin(m.declination) + std::cos(s.declination) * std::cos(m.declination) * std::cos(s.rightAscension - m.rightAscension));
        double inc = std::atan2(sdist * std::sin(phi), m.distance - sdist * std::cos(phi));
        double angle = std::atan2(std::cos(s.declination) * std::sin(s.rightAscension - m.rightAscension), std::sin(s.declination) * std::cos(m.declination) -	std::cos(s.declination) * std::sin(m.declination) * std::cos(s.rightAscension - m.rightAscension));

        CelestialPhase moonPhase;
        moonPhase.fraction = (1 + cos(inc)) / 2;
        moonPhase.phase = 0.5 + 0.5 * inc * (angle < 0 ? -1 : 1) / M_PI;
        moonPhase.angle = angle;
        return moonPhase;
    };


private:

    static double toDays(uint64_t date)
    {
        static constexpr double kJulianDaysOnY2000 = 2451545;
        static constexpr double kDaysToHours = 60 * 60 * 24;
        static constexpr double kJulianDaysOnEpoch = 2440588;

        double julian_days = date / kDaysToHours - 0.5 + kJulianDaysOnEpoch;;
        return julian_days - kJulianDaysOnY2000;
    }


    static double rightAscension(double l, double b)
    {
        return std::atan2(std::sin(l) * std::cos(EarthUtils::Obliquity) - std::tan(b) * std::sin(EarthUtils::Obliquity), std::cos(l));
    }

    static double declination(double l, double b)
    {
        return std::asin(std::sin(b) * std::cos(EarthUtils::Obliquity) + std::cos(b) * std::sin(EarthUtils::Obliquity) * std::sin(l));
    }

    static double azimuth(double H, double phi, double declination)
    {
        return std::atan2(std::sin(H), std::cos(H) * std::sin(phi) - std::tan(declination) * std::cos(phi));
    }

    static double altitude(double H, double phi, double declination)
    {
        return std::asin(std::sin(phi) * std::sin(declination) + std::cos(phi) * std::cos(declination) * std::cos(H));
    }

    static double siderealTime(double d, double lw)
    {
        return Utils::degreesToRadians((280.16 + 360.9856235 * d)) - lw;
    }

    static double astroRefraction(double h)
    {
        if (h < 0) // the following formula works for positive altitudes only.
            h = 0; // if h = -0.08901179 a div/0 would occur.

        // formula 16.4 of "Astronomical Algorithms" 2nd edition by Jean Meeus (Willmann-Bell, Richmond) 1998.
        // 1.02 / tan(h + 10.26 / (h + 5.10)) h in degrees, result in arc minutes -> converted to rad:
        return 0.0002967 / std::tan(h + 0.00312536 / (h + 0.08901179));
    }


    static double solarMeanAnomaly(double d)
    {
        return Utils::degreesToRadians((357.5291 + 0.98560028 * d));
    }

    static double eclipticLongitude(double M)
    {
        double C = Utils::degreesToRadians((1.9148 * std::sin(M) + 0.02 * std::sin(2 * M) + 0.0003 * std::sin(3 * M))); // equation of center

        return M + C + EarthUtils::Perihelion + M_PI;
    }

    static CelestialGlobalCoord getGlobalSunCoords(double d)
    {
        double M = solarMeanAnomaly(d);
        double L = eclipticLongitude(M);

        CelestialGlobalCoord sunCoords;
        sunCoords.declination = declination(L, 0);
        sunCoords.rightAscension = rightAscension(L, 0);

        return sunCoords;
    }


    // moon calculations, based on http://aa.quae.nl/en/reken/hemelpositie.html formulas
    static CelestialGlobalCoord getGlobalMoonCoords(double d)
    { 
        // geocentric ecliptic coordinates of the moon

        double L = Utils::degreesToRadians((218.316 + 13.176396 * d)); // ecliptic longitude
        double M = Utils::degreesToRadians((134.963 + 13.064993 * d)); // mean anomaly
        double F = Utils::degreesToRadians((93.272 + 13.229350 * d));  // mean distance

        double l = L + Utils::degreesToRadians(6.289 * std::sin(M)); // longitude
        double b  = Utils::degreesToRadians(5.128 * std::sin(F));     // latitude
        double dt = 385001 - 20905 * std::cos(M);  // distance to the moon in km

        CelestialGlobalCoord moonCoords;
        moonCoords.rightAscension = rightAscension(l, b);
        moonCoords.declination = declination(l, b);
        moonCoords.distance = dt;

        return moonCoords;
    }
};


}} //namespace
#endif
