#ifndef PATH_PLANNING_HPP
#define PATH_PLANNING_HPP

#include <vector>
#include <algorithm>
#include <random>
#include <chrono> // For timing
#include <iostream>

#include "gps_meter_converter.hpp"

const double SAFETY_MARGIN_METERS = 10.0; // Safety margin for path planning

// Obstacle structure
struct Obstacle {
    GPSPoint gps;
    MeterPoint meter;
    double radius;
    
    Obstacle(const GPSPoint& g, const MeterPoint& m, double r) 
        : gps(g), meter(m), radius(r) {}
};

// RRTNode structure
struct RRTNode {
    MeterPoint point;
    int parent;
    double cost;
    
    RRTNode(const MeterPoint& p, int par = -1, double c = 0.0) 
        : point(p), parent(par), cost(c) {}
};

// Collision detection functions
bool checkCollision(const MeterPoint& point, const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        double dist = point.distance(obs.meter);
        if (dist < obs.radius + SAFETY_MARGIN_METERS) {
            return true;
        }
    }
    return false;
}

bool checkPathCollision(const MeterPoint& p1, const MeterPoint& p2, 
                       const std::vector<Obstacle>& obstacles, int numChecks = 10) {
    double dist = p1.distance(p2);
    int dynamicChecks = std::max(numChecks, static_cast<int>(dist / 5.0));
    for (int i = 0; i <= dynamicChecks; ++i) {
        double t = static_cast<double>(i) / dynamicChecks;
        MeterPoint intermediate(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
        if (checkCollision(intermediate, obstacles)) {
            return true;
        }
    }
    return false;
}

// RRT* path planning algorithm
std::vector<MeterPoint> rrtStarPathPlanning(
    const MeterPoint& start,
    const MeterPoint& goal,
    const std::vector<Obstacle>& obstacles,
    double stepSize = 10.0,
    double goalRadius = 20.0,
    int maxIterations = 5000,
    double searchRadius = 50.0) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<RRTNode> nodes;
    nodes.push_back(RRTNode(start, -1, 0.0));
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    double minX = std::min(start.x, goal.x) - 200;
    double maxX = std::max(start.x, goal.x) + 200;
    double minY = std::min(start.y, goal.y) - 200;
    double maxY = std::max(start.y, goal.y) + 200;
    
    std::uniform_real_distribution<double> xDist(minX, maxX);
    std::uniform_real_distribution<double> yDist(minY, maxY);
    
    int goalNodeIdx = -1;
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        MeterPoint randomPoint;
        if (iter % 10 == 0) {
            randomPoint = goal;
        } else {
            randomPoint = MeterPoint(xDist(gen), yDist(gen));
        }
        
        int nearestIdx = 0;
        double minDist = nodes[0].point.distance(randomPoint);
        for (size_t i = 1; i < nodes.size(); ++i) {
            double dist = nodes[i].point.distance(randomPoint);
            if (dist < minDist) {
                minDist = dist;
                nearestIdx = i;
            }
        }
        
        MeterPoint nearest = nodes[nearestIdx].point;
        double dist = nearest.distance(randomPoint);
        MeterPoint newPoint;
        if (dist <= stepSize) {
            newPoint = randomPoint;
        } else {
            double ratio = stepSize / dist;
            newPoint = MeterPoint(
                nearest.x + ratio * (randomPoint.x - nearest.x),
                nearest.y + ratio * (randomPoint.y - nearest.y)
            );
        }
        
        if (checkCollision(newPoint, obstacles) || 
            checkPathCollision(nearest, newPoint, obstacles)) {
            continue;
        }
        
        double newCost = nodes[nearestIdx].cost + nearest.distance(newPoint);
        int bestParent = nearestIdx;
        
        for (size_t i = 0; i < nodes.size(); ++i) {
            double distToNew = nodes[i].point.distance(newPoint);
            if (distToNew <= searchRadius) {
                double potentialCost = nodes[i].cost + distToNew;
                if (potentialCost < newCost && 
                    !checkPathCollision(nodes[i].point, newPoint, obstacles)) {
                    newCost = potentialCost;
                    bestParent = i;
                }
            }
        }
        
        int newNodeIdx = nodes.size();
        nodes.push_back(RRTNode(newPoint, bestParent, newCost));
        
        for (size_t i = 0; i < nodes.size() - 1; ++i) {
            double distToNew = nodes[i].point.distance(newPoint);
            if (distToNew <= searchRadius) {
                double potentialCost = newCost + distToNew;
                if (potentialCost < nodes[i].cost && 
                    !checkPathCollision(newPoint, nodes[i].point, obstacles)) {
                    nodes[i].parent = newNodeIdx;
                    nodes[i].cost = potentialCost;
                }
            }
        }
        
        if (newPoint.distance(goal) < goalRadius) {
            goalNodeIdx = newNodeIdx;
            break;
        }
    }
    
    std::vector<MeterPoint> path;
    if (goalNodeIdx != -1) {
        int current = goalNodeIdx;
        while (current != -1) {
            path.push_back(nodes[current].point);
            current = nodes[current].parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    if (path.empty() && !checkPathCollision(start, goal, obstacles, 100)) {
        path.push_back(start);
        path.push_back(goal);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Path planning took " << duration.count() << " milliseconds." << std::endl;

    return path;
}

// Path simplification
std::vector<MeterPoint> simplifyPath(const std::vector<MeterPoint>& path, 
                                     const std::vector<Obstacle>& obstacles) {
    if (path.size() <= 2) return path;
    
    std::vector<MeterPoint> simplified;
    simplified.push_back(path[0]);
    
    size_t current = 0;
    while (current < path.size() - 1) {
        size_t next = path.size() - 1;
        for (size_t i = path.size() - 1; i > current; --i) {
            double dist = path[current].distance(path[i]);
            int numChecks = std::max(100, static_cast<int>(dist / 2.0));
            if (!checkPathCollision(path[current], path[i], obstacles, numChecks)) {
                next = i;
                break;
            }
        }
        simplified.push_back(path[next]);
        current = next;
    }
    
    return simplified;
}

// Structure to hold both original and simplified paths
struct PathResult {
    std::vector<GPSPoint> originalPath;
    std::vector<GPSPoint> simplifiedPath;
};

// Forward declaration
PathResult pathPlanningWithBothPaths(
    const GPSPoint& pointA,
    const GPSPoint& pointB,
    const std::vector<Obstacle>& obstacles);

// Main path planning function
std::vector<GPSPoint> pathPlanning(
    const GPSPoint& pointA,
    const GPSPoint& pointB,
    const std::vector<Obstacle>& obstacles) {
    
    PathResult result = pathPlanningWithBothPaths(pointA, pointB, obstacles);
    return result.simplifiedPath;
}

// Path planning function that returns both original and simplified paths
PathResult pathPlanningWithBothPaths(
    const GPSPoint& pointA,
    const GPSPoint& pointB,
    const std::vector<Obstacle>& obstacles) {
    
    PathResult result;
    
    MeterPoint start = gpsToMeter(pointA, pointA);
    MeterPoint goal = gpsToMeter(pointA, pointB);
    
    double maxObstacleRadius = 0.0;
    for (const auto& obs : obstacles) {
        maxObstacleRadius = std::max(maxObstacleRadius, obs.radius);
    }
    
    double stepSize = std::max(10.0, maxObstacleRadius * 0.2);
    double goalRadius = 1.0;
    int maxIterations = 10000;
    double searchRadius = std::max(50.0, maxObstacleRadius * 1.5);
    
    std::vector<MeterPoint> pathMeter = rrtStarPathPlanning(start, goal, obstacles, 
                                                           stepSize, goalRadius, 
                                                           maxIterations, searchRadius);
    
    // Convert original path to GPS
    for (const auto& point : pathMeter) {
        result.originalPath.push_back(meterToGPS(pointA, point));
    }
    
    if (!pathMeter.empty()) {
        std::vector<MeterPoint> simplifiedPath = simplifyPath(pathMeter, obstacles);
        
        bool pathValid = true;
        for (size_t i = 0; i < simplifiedPath.size() - 1; ++i) {
            double dist = simplifiedPath[i].distance(simplifiedPath[i + 1]);
            int numChecks = std::max(200, static_cast<int>(dist / 1.0));
            if (checkPathCollision(simplifiedPath[i], simplifiedPath[i + 1], obstacles, numChecks)) {
                std::cout << "Warning: Simplified path segment " << i << " to " << (i+1) 
                          << " collides with obstacles! Using original path." << std::endl;
                pathValid = false;
                break;
            }
        }
        
        if (pathValid) {
            pathMeter = simplifiedPath;
        }
    }
    
    // Convert simplified path to GPS
    for (const auto& point : pathMeter) {
        result.simplifiedPath.push_back(meterToGPS(pointA, point));
    }
    
    return result;
}

#endif // PATH_PLANNING_HPP
