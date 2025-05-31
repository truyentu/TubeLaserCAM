// GeometryWrapper/Header Files/stdafx.h
#pragma once

// --- SECTION 1: Pure C++ headers and Native nullptr context ---
// Ensure native nullptr semantics for this section.
// If nullptr was defined as a macro by some previous non-standard header (unlikely with modern compilers),
// undefining it ensures 'nullptr' refers to the C++11 keyword or is undefined if not C++11.
#ifdef nullptr
#undef nullptr
#endif

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h> // For WinAPI, used by msclr/marshal.h indirectly

// Standard C++ Library Headers
#include <vector>
#include <string>
#include <memory>
#include <map>

// OpenCASCADE Headers (as used by GeometryKernel/StepReader.h)
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
// Other OpenCASCADE headers that StepReader.cpp might use and benefit from PCH:
#include <STEPControl_Reader.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <TopExp.hxx>
#include <TopoDS_Vertex.hxx>
#include <ElCLib.hxx>
#include <BRepTools.hxx>
#include <Precision.hxx>

// Include the project's main native header file(s) here
// This ensures they are compiled with native nullptr semantics.
#include "../GeometryKernel/StepReader.h" // Now StepReader.h and its includes are processed in native context

// --- SECTION 2: C++/CLI specific setup ---
#include <vcclr.h> // For gcroot, .NET interop utilities. Must come before managed nullptr redefinition.

// Re-define nullptr for managed code contexts AFTER all native headers are processed.
#ifdef _MANAGED
  // It's possible windows.h or another header might have defined nullptr as a macro.
  // Ensure it's undefined before redefining it as __nullptr for managed code.
  #ifdef nullptr
  #undef nullptr
  #endif
  #define nullptr __nullptr
#endif

// --- SECTION 3: Managed specific headers or .NET assembly references ---
// These headers expect to be compiled in a managed context (e.g., with nullptr as __nullptr).
#include <msclr/marshal_cppstd.h> // Used in ManagedStepReader.cpp

// Example: #using <System.dll>
