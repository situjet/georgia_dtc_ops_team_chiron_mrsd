#ifndef GPS_METER_CONVERTER_HPP
#define GPS_METER_CONVERTER_HPP

#include <cmath>

// GPSPoint structure
struct GPSPoint {
    double lat; 
    double lon;
    
    GPSPoint(double latitude = 0.0, double longitude = 0.0) 
        : lat(latitude), lon(longitude) {}
};

// MeterPoint structure
struct MeterPoint {
    double x;
    double y;
    
    MeterPoint(double x_m = 0.0, double y_m = 0.0) 
        : x(x_m), y(y_m) {}
    
    double distance(const MeterPoint& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

const double METERS_PER_DEG_LAT = 111111.0;

MeterPoint gpsToMeter(const GPSPoint& reference, const GPSPoint& target) {
    double dLat = target.lat - reference.lat;
    double dLon = target.lon - reference.lon;
    
    double latDist = dLat * METERS_PER_DEG_LAT;
    double lat_rad = reference.lat * M_PI / 180.0;
    double meters_per_deg_lon = METERS_PER_DEG_LAT * std::cos(lat_rad);
    double lonDist = dLon * meters_per_deg_lon;
    
    return MeterPoint(lonDist, latDist);
}

GPSPoint meterToGPS(const GPSPoint& reference, const MeterPoint& target) {
    double lat_rad = reference.lat * M_PI / 180.0;
    double meters_per_deg_lon = METERS_PER_DEG_LAT * std::cos(lat_rad);
    
    double dLat = target.y / METERS_PER_DEG_LAT;
    double dLon = target.x / meters_per_deg_lon;
    
    return GPSPoint(reference.lat + dLat, reference.lon + dLon);
}

#endif // GPS_METER_CONVERTER_HPP
