// SPDX-FileCopyrightText: Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "core/mesh.h"

#include <fstream>

namespace mesh
{

class MeshImporter
{
private:
    // std::map<std::string, std::pair<Mesh*, carb::gym::GymMeshHandle>> gymGraphicsMeshCache;
    // std::map<std::pair<float, float>, carb::gym::TriangleMeshHandle> cylinderCache;
    // std::map<std::string, Mesh*> simulationMeshCache;

public:
    MeshImporter()
    {
    }

    std::string resolveMeshPath(const std::string& filePath)
    {
        // noop for now
        return filePath;
    }

    Mesh* loadMeshAssimp(std::string relativeMeshPath, const Vec3& scale, GymMeshNormalMode normalMode, bool flip = false)
    {
        std::string meshPath = resolveMeshPath(relativeMeshPath);

        Mesh* mesh = ImportMeshAssimp(meshPath.c_str());
        if (mesh == nullptr)
        {
            return nullptr;
        }
        if (mesh->m_positions.size() == 0)
        {
            return nullptr;
        }

        if (normalMode == GymMeshNormalMode::eFromAsset)
        {
            // asset normals should already be in the mesh
            if (mesh->m_normals.size() != mesh->m_positions.size())
            {
                // fall back to vertex norms
                mesh->CalculateNormals();
            }
        }
        else if (normalMode == GymMeshNormalMode::eComputePerVertex)
        {
            mesh->CalculateNormals();
        }
        else if (normalMode == GymMeshNormalMode::eComputePerFace)
        {
            mesh->CalculateFaceNormals();
        }

        // Use normals to generate missing texcoords
        for (unsigned int i = 0; i < mesh->m_texcoords.size(); ++i)
        {
            Vector2& uv = mesh->m_texcoords[i];
            if (uv.x < 0 && uv.y < 0)
            {
                Vector3& n = mesh->m_normals[i];
                uv.x = std::atan2(n.z, n.x) / k2Pi;
                uv.y = std::atan2(std::sqrt(n.x * n.x + n.z * n.z), n.y) / kPi;
            }
        }

        if (flip)
        {
            Matrix44 flip;
            flip(0, 0) = 1.0f;
            flip(2, 1) = 1.0f;
            flip(1, 2) = -1.0f;
            flip(3, 3) = 1.0f;
            mesh->Transform(flip);
        }

        mesh->Transform(ScaleMatrix(scale));

        return mesh;
    }

    Mesh* loadMeshFromObj(std::string relativeMeshPath, const Vec3& scale, bool flip = false)
    {
        std::string meshPath = resolveMeshPath(relativeMeshPath);

        size_t extensionPosition = meshPath.find_last_of(".");
        meshPath.replace(extensionPosition, std::string::npos, ".obj");

        Mesh* mesh = ImportMeshFromObj(meshPath.c_str());
        if (mesh == nullptr)
        {
            return nullptr;
        }
        if (mesh->m_positions.size() == 0)
        {
            // Memory leak? `Mesh` has no `delete` mechanism
            return nullptr;
        }
        if (flip)
        {
            Matrix44 flip;
            flip(0, 0) = 1.0f;
            flip(2, 1) = 1.0f;
            flip(1, 2) = -1.0f;
            flip(3, 3) = 1.0f;
            mesh->Transform(flip);
        }

        mesh->Transform(ScaleMatrix(scale));

        // use flat normals on collision shapes
        mesh->CalculateFaceNormals();

        return mesh;
    }

    Mesh* loadMeshFromWrl(std::string relativeMeshPath, const Vec3& scale)
    {
        std::string meshPath = resolveMeshPath(relativeMeshPath);

        size_t extensionPosition = meshPath.find_last_of(".");
        meshPath.replace(extensionPosition, std::string::npos, ".wrl");

        std::ifstream inf(meshPath);
        if (!inf)
        {
            printf("File %s not found!\n", meshPath.c_str());
            return nullptr;
        }

        // TODO Avoid!
        Mesh* mesh = new Mesh();
        std::string str;
        while (inf >> str)
        {
            if (str == "point")
            {
                std::vector<Vec3> points;
                std::string tmp;
                inf >> tmp;
                while (tmp != "]")
                {
                    float x, y, z;
                    std::string ss;
                    inf >> ss;
                    if (ss == "]")
                    {
                        break;
                    }
                    x = (float)atof(ss.c_str());
                    inf >> y >> z;
                    points.push_back(Vec3(x * scale.x, y * scale.y, z * scale.z));
                    inf >> tmp;
                }

                while (inf >> str)
                {
                    if (str == "coordIndex")
                    {
                        std::vector<int> indices;
                        inf >> tmp;
                        inf >> tmp;
                        while (tmp != "]")
                        {
                            int i0, i1, i2;

                            if (tmp == "]")
                            {
                                break;
                            }

                            sscanf(tmp.c_str(), "%d", &i0);

                            std::string s1, s2, s3;
                            inf >> s1 >> s2 >> s3;
                            sscanf(s1.c_str(), "%d", &i1);
                            sscanf(s2.c_str(), "%d", &i2);

                            indices.push_back(i0);
                            indices.push_back(i1);
                            indices.push_back(i2);

                            inf >> tmp;
                        }

                        // Now found triangles too, create convex

                        mesh->m_positions.resize(points.size());
                        mesh->m_indices.resize(indices.size());
                        for (size_t i = 0; i < points.size(); i++)
                        {
                            mesh->m_positions[i].x = points[i].x;
                            mesh->m_positions[i].y = points[i].y;
                            mesh->m_positions[i].z = points[i].z;
                        }

                        memcpy(&mesh->m_indices[0], &indices[0], sizeof(int) * indices.size());
                        mesh->CalculateNormals();

                        break;
                    }
                }
            }
        }
        inf.close();

        return mesh;
    }
};

} // namespace mesh
