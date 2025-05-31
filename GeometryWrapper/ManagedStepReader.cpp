#include "stdafx.h"
#include "ManagedStepReader.h"
#include "../GeometryKernel/StepReader.h"
#include <msclr/marshal_cppstd.h>

using namespace GeometryWrapper;
using namespace System::Runtime::InteropServices;

// Helper to convert native EdgeClassification::Location to managed
ManagedEdgeClassification::Location ConvertLocation(GeometryKernel::EdgeClassification::Location nativeLoc) {
    switch (nativeLoc) {
    case GeometryKernel::EdgeClassification::Location::ON_CYLINDER_SURFACE:
        return ManagedEdgeClassification::Location::OnCylinderSurface;
    case GeometryKernel::EdgeClassification::Location::ON_END_FACE:
        return ManagedEdgeClassification::Location::OnEndFace;
    case GeometryKernel::EdgeClassification::Location::INTERNAL:
        return ManagedEdgeClassification::Location::Internal;
    default:
        return ManagedEdgeClassification::Location::Unknown;
    }
}

// Helper to convert native EdgeClassification::ShapeType to managed
ManagedEdgeClassification::ShapeType ConvertShapeType(GeometryKernel::EdgeClassification::ShapeType nativeShape) {
    switch (nativeShape) {
    case GeometryKernel::EdgeClassification::ShapeType::LINE_SEGMENT:
        return ManagedEdgeClassification::ShapeType::Line;
    case GeometryKernel::EdgeClassification::ShapeType::CIRCLE_SHAPE:
        return ManagedEdgeClassification::ShapeType::Circle;
    case GeometryKernel::EdgeClassification::ShapeType::RECTANGLE_SHAPE:
        return ManagedEdgeClassification::ShapeType::Other; // Map to Other since managed doesn't have Rectangle
    case GeometryKernel::EdgeClassification::ShapeType::SQUARE_SHAPE:
        return ManagedEdgeClassification::ShapeType::Other; // Map to Other
    case GeometryKernel::EdgeClassification::ShapeType::SLOT_SHAPE:
        return ManagedEdgeClassification::ShapeType::Other; // Map to Other
    case GeometryKernel::EdgeClassification::ShapeType::FREEFORM_SHAPE:
        return ManagedEdgeClassification::ShapeType::BSpline; // Map freeform to BSpline
    case GeometryKernel::EdgeClassification::ShapeType::UNKNOWN_SHAPE:
    default:
        return ManagedEdgeClassification::ShapeType::Other;
    }
}

ManagedStepReader::ManagedStepReader() {
    m_pNativeReader = new GeometryKernel::StepReader();
}

ManagedStepReader::~ManagedStepReader() {
    if (m_pNativeReader != nullptr) {
        delete m_pNativeReader;
        m_pNativeReader = nullptr;
    }
}

ManagedStepReader::!ManagedStepReader() {
    if (m_pNativeReader != nullptr) {
        delete m_pNativeReader;
        m_pNativeReader = nullptr;
    }
}

bool ManagedStepReader::LoadFile(String^ filePath) {
    if (m_pNativeReader == nullptr) return false;
    std::string nativeFilePath = msclr::interop::marshal_as<std::string>(filePath);
    return m_pNativeReader->LoadFile(nativeFilePath);
}

int ManagedStepReader::GetEdgeCount() {
    if (m_pNativeReader == nullptr) return 0;
    return m_pNativeReader->GetEdgeCount();
}

List<ManagedEdgeInfo>^ ManagedStepReader::GetEdgeInfoList() {
    if (m_pNativeReader == nullptr) return nullptr;

    std::vector<GeometryKernel::EdgeInfo> nativeEdgeInfos = m_pNativeReader->GetEdgeInfoList();
    List<ManagedEdgeInfo>^ managedList = gcnew List<ManagedEdgeInfo>();

    for (const auto& nativeInfo : nativeEdgeInfos) {
        ManagedEdgeInfo managedInfo;
        managedInfo.Id = nativeInfo.id;
        managedInfo.Length = nativeInfo.length;

        switch (nativeInfo.type) {
        case GeometryKernel::EdgeInfo::LINE:
            managedInfo.Type = ManagedEdgeInfo::EdgeType::Line;
            break;
        case GeometryKernel::EdgeInfo::CIRCLE:
            managedInfo.Type = ManagedEdgeInfo::EdgeType::Circle;
            break;
        case GeometryKernel::EdgeInfo::BSPLINE:
            managedInfo.Type = ManagedEdgeInfo::EdgeType::BSpline;
            break;
        case GeometryKernel::EdgeInfo::BEZIER:
            managedInfo.Type = ManagedEdgeInfo::EdgeType::Bezier;
            break;
        default:
            managedInfo.Type = ManagedEdgeInfo::EdgeType::Other;
            break;
        }
        managedList->Add(managedInfo);
    }
    return managedList;
}

void ManagedStepReader::GetWireframeData(List<Point3D>^% vertices, List<Tuple<int, int>^>^% lineIndices) {
    if (m_pNativeReader == nullptr) {
        vertices = gcnew List<Point3D>();
        lineIndices = gcnew List<Tuple<int, int>^>();
        return;
    }

    std::vector<double> nativeVertices;
    std::vector<int> nativeIndices;
    m_pNativeReader->ExtractWireframeData(nativeVertices, nativeIndices);

    vertices = gcnew List<Point3D>();
    for (size_t i = 0; i < nativeVertices.size(); i += 3) {
        vertices->Add(Point3D(nativeVertices[i], nativeVertices[i + 1], nativeVertices[i + 2]));
    }

    lineIndices = gcnew List<Tuple<int, int>^>();
    for (size_t i = 0; i < nativeIndices.size(); i += 2) {
        lineIndices->Add(Tuple::Create(nativeIndices[i], nativeIndices[i + 1]));
    }
}

ManagedCylinderInfo ManagedStepReader::DetectCylinder() {
    ManagedCylinderInfo managedInfo;
    managedInfo.IsValid = false;

    if (m_pNativeReader == nullptr) return managedInfo;

    GeometryKernel::CylinderInfo nativeInfo = m_pNativeReader->DetectCylinder();

    // Debug
    System::Diagnostics::Debug::WriteLine("Native: R={0}, L={1}", nativeInfo.radius, nativeInfo.length);

    managedInfo.IsValid = nativeInfo.isValid;
    managedInfo.Radius = nativeInfo.radius;
    managedInfo.Length = nativeInfo.length;
    managedInfo.AxisX = nativeInfo.axisX;
    managedInfo.AxisY = nativeInfo.axisY;
    managedInfo.AxisZ = nativeInfo.axisZ;
    managedInfo.CenterX = nativeInfo.centerX;
    managedInfo.CenterY = nativeInfo.centerY;
    managedInfo.CenterZ = nativeInfo.centerZ;

    // Debug
    System::Diagnostics::Debug::WriteLine("Managed: R={0}, L={1}", managedInfo.Radius, managedInfo.Length);

    return managedInfo;
}

List<Point3D>^ ManagedStepReader::GetEdgePoints(int edgeId) {
    if (m_pNativeReader == nullptr) return nullptr;
    // Kiểm tra classification trước
    auto classifications = m_pNativeReader->ClassifyEdges();
    auto it = classifications.find(edgeId);
    if (it != classifications.end() &&
        it->second.location == GeometryKernel::EdgeClassification::INTERNAL) {
        return nullptr; // Không trả về points cho INTERNAL edges
    }

    std::vector<double> nativePoints;
    bool success = m_pNativeReader->GetEdgeGeometry(edgeId, nativePoints);

    if (!success) return nullptr;

    List<Point3D>^ managedPoints = gcnew List<Point3D>();
    for (size_t i = 0; i < nativePoints.size(); i += 3) {
        managedPoints->Add(Point3D(nativePoints[i], nativePoints[i + 1], nativePoints[i + 2]));
    }
    return managedPoints;
}

Dictionary<int, ManagedEdgeClassification>^ ManagedStepReader::GetEdgeClassifications(ManagedCylinderInfo cylinderInfo) {
    if (m_pNativeReader == nullptr) {
        return gcnew Dictionary<int, ManagedEdgeClassification>();
    }

    // ClassifyEdges không nhận parameter trong native code
    std::map<int, GeometryKernel::EdgeClassification> nativeClassifications = m_pNativeReader->ClassifyEdges();

    Dictionary<int, ManagedEdgeClassification>^ managedClassifications = gcnew Dictionary<int, ManagedEdgeClassification>();

    for (const auto& pair : nativeClassifications) {
        ManagedEdgeClassification managedClass;
        managedClass.OriginalEdgeId = pair.second.originalEdgeId;
        managedClass.IsOnCylinderSurface = pair.second.isOnCylinderSurface;
        managedClass.ClassificationLocation = ConvertLocation(pair.second.location);
        managedClass.TypeOfShape = ConvertShapeType(pair.second.shapeType);
        managedClassifications->Add(pair.first, managedClass);
    }

    return managedClassifications;
}

List<ManagedToolpathCandidate>^ ManagedStepReader::GetToolpathCandidates() {
    if (m_pNativeReader == nullptr) return nullptr;

    auto nativeCandidates = m_pNativeReader->GenerateToolpathCandidates();
    auto managedCandidates = gcnew List<ManagedToolpathCandidate>();

    for (const auto& native : nativeCandidates) {
        ManagedToolpathCandidate managed;
        managed.EdgeIds = gcnew array<int>(static_cast<int>(native.edgeIds.size()));
        for (size_t i = 0; i < native.edgeIds.size(); i++) {
            managed.EdgeIds[i] = native.edgeIds[i];
        }
        managed.Priority = native.priority;
        managed.Type = gcnew String(native.type.c_str());
        managed.TotalLength = native.totalLength;

        managedCandidates->Add(managed);
    }

    return managedCandidates;
}
List<List<int>^>^ ManagedStepReader::GetEdgeGroups() {
    if (m_pNativeReader == nullptr) return nullptr;
    auto groups = m_pNativeReader->GroupConnectedEdges();
    auto managedGroups = gcnew List<List<int>^>();

    for (const auto& group : groups) {
        auto managedGroup = gcnew List<int>();
        for (int id : group.edgeIds) {
            managedGroup->Add(id);
        }
        managedGroups->Add(managedGroup);
    }

    return managedGroups;
}
