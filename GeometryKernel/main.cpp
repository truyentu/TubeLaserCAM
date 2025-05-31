#include <iostream>
#include "GeometryTypes.h"
#include "StepReader.h"

int main() {
    GeometryKernel::StepReader reader;

    // Test loading a STEP file
    if (reader.LoadFile("test.step")) {
        std::cout << "File loaded successfully!" << std::endl;
        std::cout << "Number of edges: " << reader.GetEdgeCount() << std::endl;

        // Get edge information
        auto edges = reader.GetEdgeInfoList();
        for (const auto& edge : edges) {
            std::cout << "Edge " << edge.id << ": ";
            switch (edge.type) {
            case GeometryKernel::EdgeInfo::LINE:
                std::cout << "Line";
                break;
            case GeometryKernel::EdgeInfo::CIRCLE:
                std::cout << "Circle";
                break;
            case GeometryKernel::EdgeInfo::BSPLINE:
                std::cout << "B-Spline";
                break;
            case GeometryKernel::EdgeInfo::BEZIER:
                std::cout << "Bezier";
                break;
            default:
                std::cout << "Other";
                break;
            }
            std::cout << ", Length: " << edge.length << "mm" << std::endl;
        }

        // Detect cylinder
        auto cylinderInfo = reader.DetectCylinder();
        if (cylinderInfo.isValid) {
            std::cout << "\nCylinder detected!" << std::endl;
            std::cout << "Radius: " << cylinderInfo.radius << "mm" << std::endl;
            std::cout << "Length: " << cylinderInfo.length << "mm" << std::endl;
        }
    }
    else {
        std::cout << "Failed to load file!" << std::endl;
    }

    return 0;
}