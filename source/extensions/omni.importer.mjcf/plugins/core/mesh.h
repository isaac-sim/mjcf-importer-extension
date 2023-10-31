// Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#pragma once
// clang-format off
#include "../UsdPCH.h"
// clang-format on

// clang-format off
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
// clang-format on

#include "assimp/scene.h"

#include "../math/core/core.h"
#include "../math/core/maths.h"

#include <string>
#include <vector>

struct TextureData
{
    int width; // width of texture - if height == 0, then width will be the same as buffer.size()
    int height; // height of textur - if height == 0, then the buffer represents a compressed image with file type
                // corresponding to format
    std::vector<uint8_t> buffer; // r8g8b8a8 if not compressed
    std::string format; // format of the data in buffer if compressed (i.e. png, jpg, bmp)
};

// direct representation of .obj style material
struct Material
{
    std::string name;

    Vec3 Ka;
    Vec3 Kd;
    Vec3 Ks;
    Vec3 emissive;

    float Ns = 50.0f; // specular exponent
    float metallic = 0.0f;
    float specular = 0.0f;

    std::string mapKd = ""; // diffuse
    std::string mapKs = ""; // shininess
    std::string mapBump = ""; // normal
    std::string mapEnv = ""; // emissive
    std::string mapMetallic = "";

    bool hasDiffuse = false;
    bool hasSpecular = false;
    bool hasMetallic = false;
    bool hasEmissive = false;
    bool hasShininess = false;
};

struct MaterialAssignment
{
    int startTri;
    int endTri;
    int startIndices;
    int endIndices;

    int material;
};

struct UVInfo
{
    std::vector<std::vector<Vector2>> uvs;
    std::vector<unsigned int> uvStartIndices;
};

/// Used when loading meshes to determine how to load normals
enum GymMeshNormalMode
{
    eFromAsset, // try to load normals from the mesh
    eComputePerVertex, // compute per-vertex normals
    eComputePerFace, // compute per-face normals
};


struct USDMesh
{
    std::string name;
    pxr::VtArray<pxr::GfVec3f> points;
    pxr::VtArray<int> faceVertexCounts;
    pxr::VtArray<int> faceVertexIndices;
    pxr::VtArray<pxr::GfVec3f> normals; // Face varing normals
    pxr::VtArray<pxr::VtArray<pxr::GfVec2f>> uvs; // Face varing uvs
    pxr::VtArray<pxr::VtArray<pxr::GfVec3f>> colors; // Face varing colors
};

struct Mesh
{
    void AddMesh(const Mesh& m);

    uint32_t GetNumVertices() const
    {
        return uint32_t(m_positions.size());
    }
    uint32_t GetNumFaces() const
    {
        return uint32_t(m_indices.size()) / 3;
    }

    void DuplicateVertex(uint32_t i);

    void CalculateFaceNormals(); // splits mesh at vertices to calculate faceted normals (changes topology)
    void CalculateNormals();
    void Transform(const Matrix44& m);
    void Normalize(float s = 1.0f); // scale so bounds in any dimension equals s and lower bound = (0,0,0)

    void Flip();

    void GetBounds(Vector3& minExtents, Vector3& maxExtents) const;

    std::string name; // optional

    std::vector<Point3> m_positions;
    std::vector<Vector3> m_normals;
    std::vector<Vector2> m_texcoords;
    std::vector<Colour> m_colours;
    std::vector<uint32_t> m_indices;

    std::vector<Material> m_materials;
    std::vector<MaterialAssignment> m_materialAssignments;

    std::vector<USDMesh> m_usdMeshPrims;
};

// Create mesh from Assimp import
void addAssimpNodeToMesh(const aiScene* scene, const aiNode* node, aiMatrix4x4 xform, UVInfo& uvInfo, Mesh* mesh);
Mesh* ImportMeshAssimp(const char* path);

// create mesh from file
Mesh* ImportMeshFromObj(const char* path);
Mesh* ImportMeshFromPly(const char* path);
Mesh* ImportMeshFromBin(const char* path);
Mesh* ImportMeshFromStl(const char* path);

// just switches on filename
Mesh* ImportMesh(const char* path);

// save a mesh in a flat binary format
void ExportMeshToBin(const char* path, const Mesh* m);

// create procedural primitives
Mesh* CreateTriMesh(float size, float y = 0.0f);
Mesh* CreateCubeMesh();
Mesh* CreateQuadMesh(float sizex, float sizez, int gridx, int gridz);
Mesh* CreateDiscMesh(float radius, uint32_t segments);
Mesh* CreateTetrahedron(float ground = 0.0f, float height = 1.0f); // fixed but not used
Mesh* CreateSphere(int slices, int segments, float radius = 1.0f);
Mesh* CreateEllipsoid(int slices, int segments, Vec3 radiis);
Mesh* CreateCapsule(int slices, int segments, float radius = 1.0f, float halfHeight = 1.0f);
Mesh* CreateCylinder(int slices, float radius, float halfHeight, bool cap = false);
