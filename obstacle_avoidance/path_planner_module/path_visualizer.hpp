#ifndef PATH_VISUALIZER_HPP
#define PATH_VISUALIZER_HPP

#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>

#include "gps_meter_converter.hpp"
#include "path_planning.hpp"

class PathVisualizer {
private:
    cv::Mat canvasMeter;
    cv::Mat canvasGPS;
    cv::Mat combinedCanvas;
    GPSPoint referencePoint;
    double scale;
    int width, height;
    std::vector<cv::Point2f> trajectoryMeter;
    std::vector<cv::Point2f> trajectoryGPS;
    
    double minLat, maxLat, minLon, maxLon;
    double minXMeter, maxXMeter, minYMeter, maxYMeter;
    bool boundsInitialized;
    int gapWidth;
    int infoPanelHeight; // Height for the new info panel
    
    void addTrajectoryPointInternal(const GPSPoint& gps) {
        if (!boundsInitialized) return;

        MeterPoint posMeter = gpsToCanvasMeter(gps);
        trajectoryMeter.push_back(cv::Point2f(static_cast<float>(posMeter.x), 
                                               static_cast<float>(posMeter.y)));
        
        cv::Point2f posGPS = gpsToCanvasGPS(gps);
        trajectoryGPS.push_back(posGPS);
    }

    MeterPoint gpsToCanvasMeter(const GPSPoint& gps) {
        MeterPoint meter = gpsToMeter(referencePoint, gps);
        double xRange = maxXMeter - minXMeter;
        double yRange = maxYMeter - minYMeter;
        double padding = 80;
        
        if (xRange == 0 || yRange == 0) {
            return MeterPoint(padding, padding);
        }
        
        double normalizedX = (meter.x - minXMeter) / xRange;
        double normalizedY = (meter.y - minYMeter) / yRange;
        
        double x = normalizedX * (width - padding * 2) + padding;
        double y = (1.0 - normalizedY) * (height - padding * 2) + padding;
        
        return MeterPoint(x, y);
    }

    cv::Point2f gpsToCanvasGPS(const GPSPoint& gps) {
        if (!boundsInitialized) {
            return cv::Point2f(0, 0);
        }
        double latRange = maxLat - minLat;
        double lonRange = maxLon - minLon;
        double padding = 80;

        if (latRange == 0 || lonRange == 0) {
            return cv::Point2f(padding, padding);
        }
        
        double normalizedLat = (gps.lat - minLat) / latRange;
        double normalizedLon = (gps.lon - minLon) / lonRange;
        
        double x = normalizedLon * (width - padding * 2) + padding;
        double y = (1.0 - normalizedLat) * (height - padding * 2) + padding;
        
        return cv::Point2f(static_cast<float>(x), static_cast<float>(y));
    }
    
    void initializeBounds(const GPSPoint& pointA, 
                           const GPSPoint& pointB,
                           const std::vector<Obstacle>& obstacles,
                           const std::vector<GPSPoint>& path) {
        
        // GPS Bounds
        minLat = std::min(pointA.lat, pointB.lat);
        maxLat = std::max(pointA.lat, pointB.lat);
        minLon = std::min(pointA.lon, pointB.lon);
        maxLon = std::max(pointA.lon, pointB.lon);
        
        for (const auto& obs : obstacles) {
            minLat = std::min(minLat, obs.gps.lat);
            maxLat = std::max(maxLat, obs.gps.lat);
            minLon = std::min(minLon, obs.gps.lon);
            maxLon = std::max(maxLon, obs.gps.lon);
        }
        
        for (const auto& p : path) {
            minLat = std::min(minLat, p.lat);
            maxLat = std::max(maxLat, p.lat);
            minLon = std::min(minLon, p.lon);
            maxLon = std::max(maxLon, p.lon);
        }
        
        double latPadding = (maxLat - minLat) * 0.1;
        double lonPadding = (maxLon - minLon) * 0.1;

        if(latPadding == 0) latPadding = 0.0001;
        if(lonPadding == 0) lonPadding = 0.0001;
        
        minLat -= latPadding;
        maxLat += latPadding;
        minLon -= lonPadding;
        maxLon += lonPadding;

        // Meter Bounds
        MeterPoint meterA = gpsToMeter(referencePoint, pointA);
        MeterPoint meterB = gpsToMeter(referencePoint, pointB);
        minXMeter = std::min(meterA.x, meterB.x);
        maxXMeter = std::max(meterA.x, meterB.x);
        minYMeter = std::min(meterA.y, meterB.y);
        maxYMeter = std::max(meterA.y, meterB.y);
        
        for (const auto& obs : obstacles) {
            minXMeter = std::min(minXMeter, obs.meter.x - obs.radius);
            maxXMeter = std::max(maxXMeter, obs.meter.x + obs.radius);
            minYMeter = std::min(minYMeter, obs.meter.y - obs.radius);
            maxYMeter = std::max(maxYMeter, obs.meter.y + obs.radius);
        }
        
        for (const auto& p : path) {
            MeterPoint meter = gpsToMeter(referencePoint, p);
            minXMeter = std::min(minXMeter, meter.x);
            maxXMeter = std::max(maxXMeter, meter.x);
            minYMeter = std::min(minYMeter, meter.y);
            maxYMeter = std::max(maxYMeter, meter.y);
        }
        
        double xPadding = (maxXMeter - minXMeter) * 0.1;
        double yPadding = (maxYMeter - minYMeter) * 0.1;
        
        if (xPadding == 0) xPadding = 10;
        if (yPadding == 0) yPadding = 10;
        
        minXMeter -= xPadding;
        maxXMeter += xPadding;
        minYMeter -= yPadding;
        maxYMeter += yPadding;
        
        boundsInitialized = true;
    }
    
    void drawObstacleMeter(const Obstacle& obs) {
        MeterPoint center = gpsToCanvasMeter(obs.gps);
        
        double xRange = maxXMeter - minXMeter;
        double padding = 80.0;
        double canvasWidth = width - padding * 2;
        
        double radiusInPixels = (obs.radius / xRange) * canvasWidth;
        
        int radius = static_cast<int>(radiusInPixels);
        if (radius < 3) radius = 3;
        
        cv::circle(canvasMeter, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)), 
                   radius, cv::Scalar(0, 0, 255), -1);
        cv::circle(canvasMeter, cv::Point(static_cast<int>(center.x), static_cast<int>(center.y)), 
                   radius, cv::Scalar(0, 0, 200), 2);
    }

    void drawObstacleGPS(const Obstacle& obs) {
        cv::Point2f center = gpsToCanvasGPS(obs.gps);
        
        double latRange = maxLat - minLat;
        double lonRange = maxLon - minLon;
        double padding = 80.0;
        double canvasWidth = width - padding * 2;
        double canvasHeight = height - padding * 2;
        
        double latRad = referencePoint.lat * M_PI / 180.0;
        double metersPerDegLat = 111111.0;
        double metersPerDegLon = 111111.0 * std::cos(latRad);
        
        double radiusInDegLat = obs.radius / metersPerDegLat;
        double radiusInDegLon = obs.radius / metersPerDegLon;
        
        double pixelsPerDegLat = canvasHeight / latRange;
        double pixelsPerDegLon = canvasWidth / lonRange;
        
        double radiusX = radiusInDegLon * pixelsPerDegLon;
        double radiusY = radiusInDegLat * pixelsPerDegLat;
        
        int radiusXInt = static_cast<int>(std::abs(radiusX));
        int radiusYInt = static_cast<int>(std::abs(radiusY));
        
        if (radiusXInt < 3) radiusXInt = 3;
        if (radiusYInt < 3) radiusYInt = 3;
        
        cv::Point centerPt(static_cast<int>(center.x), static_cast<int>(center.y));
        cv::Size axes(radiusXInt, radiusYInt);
        
        cv::ellipse(canvasGPS, centerPt, axes, 0, 0, 360, cv::Scalar(0, 0, 255), -1);
        cv::ellipse(canvasGPS, centerPt, axes, 0, 0, 360, cv::Scalar(0, 0, 200), 2);
    }
    
    void drawPointMeter(const GPSPoint& gps, const cv::Scalar& color, int radius = 8) {
        MeterPoint pos = gpsToCanvasMeter(gps);
        cv::circle(canvasMeter, cv::Point(static_cast<int>(pos.x), static_cast<int>(pos.y)), 
                   radius, color, -1);
    }

    void drawPointGPS(const GPSPoint& gps, const cv::Scalar& color, int radius = 8) {
        cv::Point2f pos = gpsToCanvasGPS(gps);
        cv::circle(canvasGPS, cv::Point(static_cast<int>(pos.x), static_cast<int>(pos.y)), 
                   radius, color, -1);
    }
    
    void drawPathMeter(const std::vector<GPSPoint>& path, const cv::Scalar& color, int thickness = 2) {
        if (path.size() < 2) return;
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            MeterPoint p1 = gpsToCanvasMeter(path[i]);
            MeterPoint p2 = gpsToCanvasMeter(path[i + 1]);
            cv::line(canvasMeter, 
                    cv::Point(static_cast<int>(p1.x), static_cast<int>(p1.y)),
                    cv::Point(static_cast<int>(p2.x), static_cast<int>(p2.y)),
                    color, thickness);
        }
    }

    void drawPathGPS(const std::vector<GPSPoint>& path, const cv::Scalar& color, int thickness = 2) {
        if (path.size() < 2) return;
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            cv::Point2f p1 = gpsToCanvasGPS(path[i]);
            cv::Point2f p2 = gpsToCanvasGPS(path[i + 1]);
            
            cv::line(canvasGPS, 
                    cv::Point(static_cast<int>(p1.x), static_cast<int>(p1.y)),
                    cv::Point(static_cast<int>(p2.x), static_cast<int>(p2.y)),
                    color, thickness);
        }
    }
    
    void drawTrajectoryMeter(const cv::Scalar& color) {
        if (trajectoryMeter.size() < 2) return;
        
        for (size_t i = 0; i < trajectoryMeter.size() - 1; ++i) {
            cv::line(canvasMeter, trajectoryMeter[i], trajectoryMeter[i + 1], color, 3);
        }
    }

    void drawTrajectoryGPS(const cv::Scalar& color) {
        if (trajectoryGPS.size() < 2) return;
        
        for (size_t i = 0; i < trajectoryGPS.size() - 1; ++i) {
            cv::line(canvasGPS, trajectoryGPS[i], trajectoryGPS[i + 1], color, 3);
        }
    }
    
    void drawAxesMeter() {
        int padding = 80;
        
        int numTicks = 5;
        for (int i = 0; i <= numTicks; ++i) {
            double xVal = minXMeter + (maxXMeter - minXMeter) * i / numTicks;
            double yVal = minYMeter + (maxYMeter - minYMeter) * i / numTicks;
            
            int xPos = padding + (width - padding * 2) * i / numTicks;
            int yPos = height - padding - (height - padding * 2) * i / numTicks;
            
            cv::line(canvasMeter, cv::Point(xPos, padding), cv::Point(xPos, height - padding), cv::Scalar(50, 50, 50), 1);
            cv::line(canvasMeter, cv::Point(padding, yPos), cv::Point(width - padding, yPos), cv::Scalar(50, 50, 50), 1);
            
            std::ostringstream ossX, ossY;
            ossX << std::fixed << std::setprecision(0) << xVal;
            cv::putText(canvasMeter, ossX.str(), cv::Point(xPos - 15, height - padding + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
            
            ossY << std::fixed << std::setprecision(0) << yVal;
            cv::putText(canvasMeter, ossY.str(), cv::Point(padding - 40, yPos + 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        }
        
        cv::rectangle(canvasMeter, cv::Point(padding, padding), cv::Point(width-padding, height-padding), cv::Scalar(255,255,255), 1);

        cv::putText(canvasMeter, "X (m)", cv::Point(width - padding - 30, height - padding + 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvasMeter, "Y (m)", cv::Point(padding - 40, padding - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    }

    void drawAxesGPS() {
        int padding = 80;

        int numTicks = 5;
        for (int i = 0; i <= numTicks; ++i) {
            double lonVal = minLon + (maxLon - minLon) * i / numTicks;
            double latVal = minLat + (maxLat - minLat) * i / numTicks;

            int xPos = padding + (width - padding * 2) * i / numTicks;
            int yPos = height - padding - (height - padding * 2) * i / numTicks;

            cv::line(canvasGPS, cv::Point(xPos, padding), cv::Point(xPos, height - padding), cv::Scalar(50, 50, 50), 1);
            cv::line(canvasGPS, cv::Point(padding, yPos), cv::Point(width - padding, yPos), cv::Scalar(50, 50, 50), 1);

            std::ostringstream ossLon, ossLat;
            ossLon << std::fixed << std::setprecision(5) << lonVal;
            cv::putText(canvasGPS, ossLon.str(), cv::Point(xPos - 30, height - padding + 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
            
            ossLat << std::fixed << std::setprecision(5) << latVal;
            cv::putText(canvasGPS, ossLat.str(), cv::Point(padding - 75, yPos + 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        }

        cv::rectangle(canvasGPS, cv::Point(padding, padding), cv::Point(width - padding, height - padding), cv::Scalar(255, 255, 255), 1);

        cv::putText(canvasGPS, "Longitude", cv::Point(width - padding - 70, height - padding + 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        cv::putText(canvasGPS, "Latitude", cv::Point(padding - 70, padding - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    }
    
    void drawInfoPanel(const GPSPoint& currentAircraftGPS, const std::vector<Obstacle>& obstacles) {
        // Create a black panel at the bottom of the combined canvas
        cv::Mat infoPanel = combinedCanvas(cv::Rect(0, height, combinedCanvas.cols, infoPanelHeight));
        infoPanel.setTo(cv::Scalar(0, 0, 0)); // Use setTo to clear the panel area

        // Calculate distances and prepare text
        MeterPoint droneMeterPos = gpsToMeter(referencePoint, currentAircraftGPS);
        
        std::vector<std::string> info_texts;
        for (size_t i = 0; i < obstacles.size(); ++i) {
            double distToCenter = droneMeterPos.distance(obstacles[i].meter);
            double distToBoundary = distToCenter - obstacles[i].radius;
            
            std::ostringstream oss;
            oss << "Drone to Obs" << i + 1 << " Boundary: " << std::fixed << std::setprecision(1) << distToBoundary << "m";
            info_texts.push_back(oss.str());
        }

        // Display text, centered
        double font_scale = 1.0;
        int thickness = 2;
        int baseline = 0;
        int total_text_width = 0;
        int text_padding = 50; // Space between text elements

        std::vector<int> text_widths;
        for(const auto& text : info_texts) {
            cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
            text_widths.push_back(text_size.width);
            total_text_width += text_size.width;
        }

        if (!info_texts.empty()) {
            total_text_width += (info_texts.size() - 1) * text_padding;
        }

        int start_x = (infoPanel.cols - total_text_width) / 2;
        if (start_x < 10) start_x = 10; // Margin protection
        
        int y_pos = (infoPanel.rows / 2) + 10; // Vertically center

        for (size_t i = 0; i < info_texts.size(); ++i) {
            cv::putText(infoPanel, info_texts[i], cv::Point(start_x, y_pos), 
                        cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255), thickness);
            start_x += text_widths[i] + text_padding;
        }
    }

    void addLabels(const GPSPoint& pointA, 
                 const GPSPoint& pointB,
                 const std::vector<Obstacle>& obstacles) {
        // Meters View
        cv::putText(canvasMeter, "Meters View", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        
        MeterPoint posAMeter = gpsToCanvasMeter(pointA);
        MeterPoint posBMeter = gpsToCanvasMeter(pointB);
        
        std::ostringstream oss;
        oss << "A: (" << std::fixed << std::setprecision(2) 
            << gpsToMeter(referencePoint, pointA).x << ", " 
            << gpsToMeter(referencePoint, pointA).y << ")";
        cv::putText(canvasMeter, oss.str(), 
                   cv::Point(static_cast<int>(posAMeter.x) + 15, static_cast<int>(posAMeter.y) - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        
        oss.str("");
        oss << "B: (" << std::fixed << std::setprecision(2) 
            << gpsToMeter(referencePoint, pointB).x << ", " 
            << gpsToMeter(referencePoint, pointB).y << ")";
        cv::putText(canvasMeter, oss.str(), 
                   cv::Point(static_cast<int>(posBMeter.x) + 15, static_cast<int>(posBMeter.y) - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

        for (size_t i = 0; i < obstacles.size(); ++i) {
            MeterPoint center = gpsToCanvasMeter(obstacles[i].gps);
            oss.str("");
            oss << "Obs" << i + 1 << ": r=" << std::fixed << std::setprecision(1) << obstacles[i].radius << "m";
            cv::putText(canvasMeter, oss.str(), 
                        cv::Point(static_cast<int>(center.x) + 10, static_cast<int>(center.y) - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        }

        // GPS View
        cv::putText(canvasGPS, "GPS View", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        
        cv::Point2f posAGPS = gpsToCanvasGPS(pointA);
        cv::Point2f posBGPS = gpsToCanvasGPS(pointB);

        oss.str("");
        oss << "A: (" << std::fixed << std::setprecision(6) 
            << pointA.lat << ", " << pointA.lon << ")";
        cv::putText(canvasGPS, oss.str(), 
                   cv::Point(static_cast<int>(posAGPS.x) + 15, static_cast<int>(posAGPS.y) - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        oss.str("");
        oss << "B: (" << std::fixed << std::setprecision(6) 
            << pointB.lat << ", " << pointB.lon << ")";
        cv::putText(canvasGPS, oss.str(), 
                   cv::Point(static_cast<int>(posBGPS.x) + 15, static_cast<int>(posBGPS.y) - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);

        for (size_t i = 0; i < obstacles.size(); ++i) {
            cv::Point2f center = gpsToCanvasGPS(obstacles[i].gps);
            oss.str("");
            oss << "Obs" << i + 1 << ": r=" << std::fixed << std::setprecision(1) << obstacles[i].radius << "m";
            cv::putText(canvasGPS, oss.str(), 
                        cv::Point(static_cast<int>(center.x) + 10, static_cast<int>(center.y) - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        }
    }

public:
    PathVisualizer(int w = 800, int h = 800, double s = 0.5) 
        : width(w), height(h), scale(s), boundsInitialized(false), gapWidth(50), infoPanelHeight(80) {
        canvasMeter = cv::Mat::zeros(height, width, CV_8UC3);
        canvasGPS = cv::Mat::zeros(height, width, CV_8UC3);
        combinedCanvas = cv::Mat::zeros(height + infoPanelHeight, width * 2 + gapWidth, CV_8UC3);
    }
    
    void setReference(const GPSPoint& ref) {
        referencePoint = ref;
    }
    
    void update(const GPSPoint& pointA, 
                const GPSPoint& pointB,
                const std::vector<Obstacle>& obstacles,
                const std::vector<GPSPoint>& originalPath,
                const std::vector<GPSPoint>& simplifiedPath,
                const GPSPoint& currentAircraftGPS) {
        
        if (!boundsInitialized) {
            std::vector<GPSPoint> combinedPath = originalPath;
            if (!simplifiedPath.empty()) {
                combinedPath.insert(combinedPath.end(), simplifiedPath.begin(), simplifiedPath.end());
            }
            initializeBounds(pointA, pointB, obstacles, combinedPath);
        }
        
        addTrajectoryPointInternal(currentAircraftGPS);

        canvasMeter = cv::Mat::zeros(height, width, CV_8UC3);
        canvasGPS = cv::Mat::zeros(height, width, CV_8UC3);
        combinedCanvas = cv::Mat::zeros(height + infoPanelHeight, width * 2 + gapWidth, CV_8UC3);
        
        drawAxesMeter();
        drawAxesGPS();
        
        for (const auto& obs : obstacles) {
            drawObstacleMeter(obs);
            drawObstacleGPS(obs);
        }
        
        drawPointMeter(pointA, cv::Scalar(0, 255, 0), 10);
        drawPointGPS(pointA, cv::Scalar(0, 255, 0), 10);
        
        drawPointMeter(pointB, cv::Scalar(255, 0, 0), 10);
        drawPointGPS(pointB, cv::Scalar(255, 0, 0), 10);
        
        // Draw original RRT* path in yellow
        drawPathMeter(originalPath, cv::Scalar(0, 255, 255), 2);
        drawPathGPS(originalPath, cv::Scalar(0, 255, 255), 2);
        
        // Draw trajectory (simulated flight) in white
        drawTrajectoryMeter(cv::Scalar(255, 255, 255));
        drawTrajectoryGPS(cv::Scalar(255, 255, 255));
        
        // Draw green circles for the simplified path waypoints
        for (const auto& waypoint : simplifiedPath) {
            drawPointMeter(waypoint, cv::Scalar(0, 255, 0), 7);
            drawPointGPS(waypoint, cv::Scalar(0, 255, 0), 7);
        }

        drawPointMeter(currentAircraftGPS, cv::Scalar(255, 255, 255), 6);
        drawPointGPS(currentAircraftGPS, cv::Scalar(255, 255, 255), 6);
        
        addLabels(pointA, pointB, obstacles);

        // Combine canvases
        cv::Mat gap = cv::Mat::zeros(height, gapWidth, CV_8UC3);
        cv::Mat combinedCanvasViews;
        cv::hconcat(canvasMeter, gap, combinedCanvasViews);
        cv::Mat temp;
        cv::hconcat(combinedCanvasViews, canvasGPS, temp);
        
        // Place the combined views onto the main canvas
        temp.copyTo(combinedCanvas(cv::Rect(0, 0, temp.cols, temp.rows)));

        // Draw the info panel at the bottom
        drawInfoPanel(currentAircraftGPS, obstacles);
    }
    
    void show(const std::string& windowName = "Path Planning Visualization") {
        cv::imshow(windowName, combinedCanvas);
    }
    
    void clearTrajectory() {
        trajectoryMeter.clear();
        trajectoryGPS.clear();
    }
};

#endif // PATH_VISUALIZER_HPP
