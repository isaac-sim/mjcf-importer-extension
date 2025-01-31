// SPDX-FileCopyrightText: Copyright (c) 2022-2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
// clang-format off
#include "../UsdPCH.h"
// clang-format on

#include "../math/core/core.h"
#include "../math/core/maths.h"
#include "omniverse_asset_converter.h"

#include <string>
#include <vector>

struct TextureData
{
    int width; // width of texture - if height == 0, then width will be the same
               // as buffer.size()
    int height; // height of textur - if height == 0, then the buffer represents a
                // compressed image with file type corresponding to format
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

    void CalculateFaceNormals(); // splits mesh at vertices to calculate faceted
                                 // normals (changes topology)
    void CalculateNormals();
    void Transform(const Matrix44& m);
    void Normalize(float s = 1.0f); // scale so bounds in any dimension equals s
                                    // and lower bound = (0,0,0)

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
    std::string m_convertedUsdMesh;
    Vec3 scale = { 1.0f, 1.0f, 1.0f };
    OmniConverterFuture* m_assetConvertStatus = nullptr;
};


// save a mesh in a flat binary format
void ExportMeshToBin(const char* path, const Mesh* m);

// create procedural primitives
Mesh* CreateTriMesh(float size, float y = 0.0f);
Mesh* CreateCubeMesh();
Mesh* CreateQuadMesh(float sizex, float sizez, int gridx, int gridz);
Mesh* CreateDiscMesh(float radius, uint32_t segments);
Mesh* CreateTetrahedron(float ground = 0.0f,
                        float height = 1.0f); // fixed but not used
Mesh* CreateSphere(int slices, int segments, float radius = 1.0f);
Mesh* CreateEllipsoid(int slices, int segments, Vec3 radiis);
Mesh* CreateCapsule(int slices, int segments, float radius = 1.0f, float halfHeight = 1.0f);
Mesh* CreateCylinder(int slices, float radius, float halfHeight, bool cap = false);
