#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <iostream>
#include <opencv2/opencv.hpp>

#include "path_planner_module/gps_meter_converter.hpp"
#include "path_planner_module/path_planning.hpp"
#include "path_planner_module/path_visualizer.hpp"

GPSPoint simulateAircraftFlight(
    const std::vector<GPSPoint>& path,
    double currentProgress,
    GPSPoint& currentGPS) {
    
    if (path.empty()) return currentGPS;
    if (path.size() == 1) return path[0];
    double totalLength = 0.0;
    std::vector<double> segmentLengths;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        MeterPoint p1 = gpsToMeter(path[0], path[i]);
        MeterPoint p2 = gpsToMeter(path[0], path[i + 1]);
        double len = p1.distance(p2);
        segmentLengths.push_back(len);
        totalLength += len;
    }
    
    double targetDistance = currentProgress * totalLength;
    double accumulated = 0.0;
    
    for (size_t i = 0; i < segmentLengths.size(); ++i) {
        if (accumulated + segmentLengths[i] >= targetDistance) {
            double t = (targetDistance - accumulated) / segmentLengths[i];
            MeterPoint p1 = gpsToMeter(path[0], path[i]);
            MeterPoint p2 = gpsToMeter(path[0], path[i + 1]);
            MeterPoint currentMeter(p1.x + t * (p2.x - p1.x), 
                                   p1.y + t * (p2.y - p1.y));
            currentGPS = meterToGPS(path[0], currentMeter);
            return currentGPS;
        }
        accumulated += segmentLengths[i];
    }
    
    currentGPS = path.back();
    return currentGPS;
}

int main() {
    GPSPoint pointA(33.7490, -84.3880);
    GPSPoint pointB(33.7550, -84.3800);
    
    GPSPoint obstacle1GPS(33.7510, -84.3840);
    GPSPoint obstacle2GPS(33.7520, -84.38000);
    
    double obstacle1Radius = 150.0;
    double obstacle2Radius = 100.0;
    
    MeterPoint obstacle1Meter = gpsToMeter(pointA, obstacle1GPS);
    MeterPoint obstacle2Meter = gpsToMeter(pointA, obstacle2GPS);
    
    std::vector<Obstacle> obstacles;
    obstacles.push_back(Obstacle(obstacle1GPS, obstacle1Meter, obstacle1Radius));
    obstacles.push_back(Obstacle(obstacle2GPS, obstacle2Meter, obstacle2Radius));
    
    PathResult pathResult = pathPlanningWithBothPaths(pointA, pointB, obstacles);
    std::vector<GPSPoint> path = pathResult.simplifiedPath;

    if (path.empty()) {
        std::cerr << "Simplified path is empty, falling back to original path." << std::endl;
        path = pathResult.originalPath;
    }

    std::cout << "Original path size: " << pathResult.originalPath.size() << std::endl;
    std::cout << "Simplified path size: " << pathResult.simplifiedPath.size() << std::endl;
    
    PathVisualizer visualizer(1000, 1000, 2.0);
    visualizer.setReference(pointA);
    
    // Clear any previous trajectory data before starting simulation
    visualizer.clearTrajectory();

    GPSPoint currentAircraftGPS = pointA;
    double flightProgress = 0.0;
    const double flightSpeed = 0.005;
    
    while (true) {
        simulateAircraftFlight(path, flightProgress, currentAircraftGPS);
        
        visualizer.update(pointA, pointB, obstacles, pathResult.originalPath, pathResult.simplifiedPath, currentAircraftGPS);
        visualizer.show();
        
        flightProgress += flightSpeed;
        if (flightProgress >= 1.0) {
            flightProgress = 1.0;
        }
        
        int key = cv::waitKey(30) & 0xFF;
        if (key == 27) {
            break;
        }
        
        if (flightProgress >= 1.0) {
            cv::waitKey(2000);
            break;
        }
    }
    
    cv::destroyAllWindows();
    return 0;
}

