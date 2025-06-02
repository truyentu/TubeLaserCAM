#include "stdafx.h"
#pragma warning(push)
#pragma warning(disable: 4635) // XML documentation comments
#pragma warning(disable: 4638) // XML documentation comments

#pragma managed(push, off)
#include <exception>
#pragma managed(pop)
// Push to unmanaged for STL/Native headers
#pragma managed(push, off)
#include "../GeometryKernel/GeometryTypes.h"  // Include TRƯỚC
#include "../GeometryKernel/StepReader.h"
#include <set>
#include <vector>
#include <map>
#pragma managed(pop)

// Managed headers
#include "ManagedStepReader.h"
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

List<ManagedUnrolledPoint^>^ ManagedStepReader::UnrollEdge(
    int edgeId,
    ManagedCylinderInfo^ cylinderInfo,
    ManagedUnrollingParams^ params) {

    // Convert managed params to native
    GeometryKernel::UnrollingParams nativeParams;
    nativeParams.chordTolerance = params->ChordTolerance;
    nativeParams.angleTolerance = params->AngleTolerance;
    nativeParams.minPointsPerCurve = params->MinPoints;
    nativeParams.maxPointsPerCurve = params->MaxPoints;
    nativeParams.unwrapAngles = params->UnwrapAngles;

    // Convert cylinder info
    GeometryKernel::CylinderInfo nativeCylinder;
    nativeCylinder.radius = cylinderInfo->Radius;
    nativeCylinder.length = cylinderInfo->Length;
    nativeCylinder.axisX = cylinderInfo->AxisX;
    nativeCylinder.axisY = cylinderInfo->AxisY;
    nativeCylinder.axisZ = cylinderInfo->AxisZ;
    nativeCylinder.centerX = cylinderInfo->CenterX;
    nativeCylinder.centerY = cylinderInfo->CenterY;
    nativeCylinder.centerZ = cylinderInfo->CenterZ;
    nativeCylinder.isValid = cylinderInfo->IsValid;

    // Call native unroll - SỬA DÒNG NÀY
    std::vector<GeometryKernel::UnrolledPoint> nativePoints =
        m_pNativeReader->UnrollEdge(edgeId, nativeCylinder, nativeParams);

    // Convert to managed list
    List<ManagedUnrolledPoint^>^ managedPoints =
        gcnew List<ManagedUnrolledPoint^>();

    for (const auto& pt : nativePoints) {
        ManagedUnrolledPoint^ managedPt = gcnew ManagedUnrolledPoint();
        managedPt->Y = pt.Y;
        managedPt->C = pt.C;
        managedPt->X = pt.X;
        managedPoints->Add(managedPt);
    }

    return managedPoints;
}
ManagedCylinderInfo ManagedStepReader::DetectMainCylinder() {
    ManagedCylinderInfo managedInfo;
    managedInfo.IsValid = false;

    if (m_pNativeReader == nullptr) return managedInfo;

    GeometryKernel::CylinderInfo nativeInfo = m_pNativeReader->DetectMainCylinder();

    managedInfo.IsValid = nativeInfo.isValid;
    managedInfo.Radius = nativeInfo.radius;
    managedInfo.Length = nativeInfo.length;
    managedInfo.AxisX = nativeInfo.axisX;
    managedInfo.AxisY = nativeInfo.axisY;
    managedInfo.AxisZ = nativeInfo.axisZ;
    managedInfo.CenterX = nativeInfo.centerX;
    managedInfo.CenterY = nativeInfo.centerY;
    managedInfo.CenterZ = nativeInfo.centerZ;

    return managedInfo;
}

String^ ManagedStepReader::ConvertProfileTypeToString(int typeValue) {
    // Dùng integer values thay vì enum
    switch (typeValue) {
    case 0:  // UNKNOWN
        return "UNKNOWN";
    case 1:  // SINGLE_CIRCLE
        return "CIRCLE";
    case 2:  // RECTANGLE
        return "RECTANGLE";
    case 3:  // SLOT
        return "SLOT";
    case 4:  // POLYGON
        return "POLYGON";
    case 5:  // COMPLEX_CLOSED
        return "COMPLEX_CLOSED";
    case 6:  // OPEN_CHAIN
        return "OPEN_CHAIN";
    default:
        return "UNKNOWN";
    }
}

// Detect complete profile from a starting edge
ManagedProfileInfo ManagedStepReader::DetectCompleteProfile(int edgeId) {
    // Initialize with invalid profile
    ManagedProfileInfo managedProfile(false);

    // Check if native reader exists
    if (m_pNativeReader == nullptr) {
        System::Diagnostics::Debug::WriteLine("Native reader is null");
        return managedProfile;
    }

    try {
        // Call native method
        auto nativeProfile = m_pNativeReader->DetectCompleteProfile(edgeId);

        // Check if profile is valid
        if (!nativeProfile.isValid) {
            System::Diagnostics::Debug::WriteLine(
                String::Format("No valid profile found for edge {0}", edgeId));
            return managedProfile;
        }

        // Convert basic profile information
        managedProfile.IsValid = nativeProfile.isValid;
        managedProfile.IsClosed = nativeProfile.isClosed;
        managedProfile.TotalLength = nativeProfile.totalLength;
        managedProfile.EdgeCount = static_cast<int>(nativeProfile.orderedEdgeIds.size());

        // THÊM: Convert gap-related information
        managedProfile.TotalGapLength = nativeProfile.totalGapLength;
        managedProfile.ProfileConfidence = nativeProfile.profileConfidence;
        managedProfile.HasVirtualEdges = nativeProfile.hasVirtualEdges;

        // Convert profile type
        managedProfile.ProfileType = ConvertProfileTypeToString(
            static_cast<int>(nativeProfile.profileType));

        // Convert edge IDs vector to managed array
        if (nativeProfile.orderedEdgeIds.size() > 0) {
            managedProfile.OrderedEdgeIds =
                gcnew array<int>(static_cast<int>(nativeProfile.orderedEdgeIds.size()));

            for (size_t i = 0; i < nativeProfile.orderedEdgeIds.size(); i++) {
                managedProfile.OrderedEdgeIds[i] = nativeProfile.orderedEdgeIds[i];
            }
        }
        else {
            managedProfile.OrderedEdgeIds = gcnew array<int>(0);
        }

        // THÊM: Convert gaps array if present
        if (nativeProfile.gaps.size() > 0) {
            managedProfile.Gaps = gcnew array<ManagedGapInfo>(
                static_cast<int>(nativeProfile.gaps.size()));

            for (size_t i = 0; i < nativeProfile.gaps.size(); i++) {
                ManagedGapInfo managedGap;
                managedGap.FromEdgeId = nativeProfile.gaps[i].fromEdgeId;
                managedGap.ToEdgeId = nativeProfile.gaps[i].toEdgeId;
                managedGap.GapDistance = nativeProfile.gaps[i].gapDistance;
                managedGap.Confidence = nativeProfile.gaps[i].confidence;

                // Convert gap closing method enum to string
                switch (nativeProfile.gaps[i].suggestedMethod) {
                case GeometryKernel::GapInfo::GapClosingMethod::VIRTUAL_LINE:  // Thêm GapClosingMethod::
                    managedGap.SuggestedMethod = "LINE";
                    break;
                case GeometryKernel::GapInfo::GapClosingMethod::VIRTUAL_ARC:   // Thêm GapClosingMethod::
                    managedGap.SuggestedMethod = "ARC";
                    break;
                case GeometryKernel::GapInfo::GapClosingMethod::EXTEND_EDGES:  // Thêm GapClosingMethod::
                    managedGap.SuggestedMethod = "EXTEND";
                    break;
                default:
                    managedGap.SuggestedMethod = "NONE";
                    break;
                }

                managedProfile.Gaps[i] = managedGap;
            }
        }
        else {
            // Initialize empty gaps array
            managedProfile.Gaps = gcnew array<ManagedGapInfo>(0);
        }

        // Enhanced debug output with gap information
        System::Diagnostics::Debug::WriteLine(
            String::Format("Profile detected: Type={0}, Edges={1}, Closed={2}, Length={3:F2}mm",
                managedProfile.ProfileType,
                managedProfile.EdgeCount,
                managedProfile.IsClosed,
                managedProfile.TotalLength));

        // THÊM: Debug output for gaps
        if (managedProfile.Gaps != nullptr && managedProfile.Gaps->Length > 0) {
            System::Diagnostics::Debug::WriteLine(
                String::Format("  Gaps: {0}, Total gap length: {1:F2}mm, Confidence: {2:F2}%",
                    managedProfile.Gaps->Length,
                    managedProfile.TotalGapLength,
                    managedProfile.ProfileConfidence * 100));

            // Log individual gaps for debugging
            for (int i = 0; i < managedProfile.Gaps->Length; i++) {
                System::Diagnostics::Debug::WriteLine(
                    String::Format("    Gap {0}: Edge {1} -> {2}, Distance: {3:F2}mm, Method: {4}",
                        i + 1,
                        managedProfile.Gaps[i].FromEdgeId,
                        managedProfile.Gaps[i].ToEdgeId,
                        managedProfile.Gaps[i].GapDistance,
                        managedProfile.Gaps[i].SuggestedMethod));
            }
        }

        // THÊM: Warning if profile has virtual edges
        if (managedProfile.HasVirtualEdges) {
            System::Diagnostics::Debug::WriteLine(
                "  Warning: Profile requires virtual edges for gap closing");
        }

    }
    catch (const std::exception& e) {
        System::String^ errorMsg = gcnew System::String(e.what());
        System::Diagnostics::Debug::WriteLine(
            String::Format("Native error in DetectCompleteProfile: {0}", errorMsg));
        managedProfile.IsValid = false;
    }
    catch (System::Exception^ e) {
        System::Diagnostics::Debug::WriteLine(
            "Managed error in DetectCompleteProfile: " + e->Message);
        managedProfile.IsValid = false;
    }

    return managedProfile;
}

// Detect all profiles in the model
List<ManagedProfileInfo>^ ManagedStepReader::DetectAllProfiles() {
    List<ManagedProfileInfo>^ managedProfiles = gcnew List<ManagedProfileInfo>();

    if (m_pNativeReader == nullptr) {
        System::Diagnostics::Debug::WriteLine("Native reader is null in DetectAllProfiles");
        return managedProfiles;
    }

    try {
        // Get all edges first
        std::vector<GeometryKernel::EdgeInfo> edges = m_pNativeReader->GetEdgeInfoList();
        std::set<int> processedEdges;

        // Process each edge
        for (const auto& edge : edges) {
            // Skip if already processed
            if (processedEdges.find(edge.id) != processedEdges.end()) {
                continue;
            }

            // Try to detect profile from this edge
            auto nativeProfile =

                m_pNativeReader->DetectCompleteProfile(edge.id);

            if (nativeProfile.isValid && nativeProfile.orderedEdgeIds.size() > 0) {
                // Convert to managed
                ManagedProfileInfo managedProfile(true);
                managedProfile.IsClosed = nativeProfile.isClosed;
                managedProfile.TotalLength = nativeProfile.totalLength;
                managedProfile.EdgeCount = static_cast<int>(nativeProfile.orderedEdgeIds.size());
                managedProfile.ProfileType = ConvertProfileTypeToString(
                    static_cast<int>(nativeProfile.profileType));

                // Convert edge IDs
                managedProfile.OrderedEdgeIds =
                    gcnew array<int>(static_cast<int>(nativeProfile.orderedEdgeIds.size()));

                for (size_t i = 0; i < nativeProfile.orderedEdgeIds.size(); i++) {
                    managedProfile.OrderedEdgeIds[i] = nativeProfile.orderedEdgeIds[i];
                    processedEdges.insert(nativeProfile.orderedEdgeIds[i]);
                }

                // Add to list
                managedProfiles->Add(managedProfile);
            }
        }

        System::Diagnostics::Debug::WriteLine(
            String::Format("Detected {0} profiles total", managedProfiles->Count));

    }
    catch (const std::exception& e) {
        System::Diagnostics::Debug::WriteLine("Native error in DetectAllProfiles");
    }
    catch (System::Exception^ e) {
        System::Diagnostics::Debug::WriteLine("Managed error in DetectAllProfiles: " + e->Message);
    }

    return managedProfiles;
}


