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

#include "mesh.h"

#include "platform.h"

#include <fstream>
#include <iostream>
#include <map>

using namespace std;

void Mesh::DuplicateVertex(uint32_t i)
{
    assert(m_positions.size() > i);
    m_positions.push_back(m_positions[i]);

    if (m_normals.size() > i)
        m_normals.push_back(m_normals[i]);

    if (m_colours.size() > i)
        m_colours.push_back(m_colours[i]);

    if (m_texcoords.size() > i)
        m_texcoords.push_back(m_texcoords[i]);
}

void Mesh::Normalize(float s)
{
    Vec3 lower, upper;
    GetBounds(lower, upper);
    Vec3 edges = upper - lower;

    Transform(TranslationMatrix(Point3(-lower)));

    float maxEdge = max(edges.x, max(edges.y, edges.z));
    Transform(ScaleMatrix(s / maxEdge));
}

void Mesh::CalculateFaceNormals()
{
    Mesh m;

    int numTris = int(GetNumFaces());

    for (int i = 0; i < numTris; ++i)
    {
        int a = m_indices[i * 3 + 0];
        int b = m_indices[i * 3 + 1];
        int c = m_indices[i * 3 + 2];

        int start = int(m.m_positions.size());

        m.m_positions.push_back(m_positions[a]);
        m.m_positions.push_back(m_positions[b]);
        m.m_positions.push_back(m_positions[c]);

        if (!m_texcoords.empty())
        {
            m.m_texcoords.push_back(m_texcoords[a]);
            m.m_texcoords.push_back(m_texcoords[b]);
            m.m_texcoords.push_back(m_texcoords[c]);
        }

        if (!m_colours.empty())
        {
            m.m_colours.push_back(m_colours[a]);
            m.m_colours.push_back(m_colours[b]);
            m.m_colours.push_back(m_colours[c]);
        }

        m.m_indices.push_back(start + 0);
        m.m_indices.push_back(start + 1);
        m.m_indices.push_back(start + 2);
    }

    m.CalculateNormals();
    m.m_materials = this->m_materials;
    m.m_materialAssignments = this->m_materialAssignments;
    m.m_usdMeshPrims = this->m_usdMeshPrims;

    *this = m;
}

void Mesh::CalculateNormals()
{
    m_normals.resize(0);
    m_normals.resize(m_positions.size());

    int numTris = int(GetNumFaces());

    for (int i = 0; i < numTris; ++i)
    {
        int a = m_indices[i * 3 + 0];
        int b = m_indices[i * 3 + 1];
        int c = m_indices[i * 3 + 2];

        Vec3 n = Cross(m_positions[b] - m_positions[a], m_positions[c] - m_positions[a]);

        m_normals[a] += n;
        m_normals[b] += n;
        m_normals[c] += n;
    }

    int numVertices = int(GetNumVertices());

    for (int i = 0; i < numVertices; ++i)
        m_normals[i] = ::Normalize(m_normals[i]);
}

namespace
{

enum PlyFormat
{
    eAscii,
    eBinaryBigEndian
};

template <typename T>
T PlyRead(ifstream& s, PlyFormat format)
{
    T data = eAscii;

    switch (format)
    {
    case eAscii:
    {
        s >> data;
        break;
    }
    case eBinaryBigEndian:
    {
        char c[sizeof(T)];
        s.read(c, sizeof(T));
        reverse(c, c + sizeof(T));
        data = *(T*)c;
        break;
    }
    default:
        assert(0);
    }

    return data;
}

} // namespace


void ExportMeshToBin(const char* path, const Mesh* m)
{
    FILE* f = fopen(path, "wb");

    if (f)
    {
        int numVertices = int(m->m_positions.size());
        int numIndices = int(m->m_indices.size());

        fwrite(&numVertices, sizeof(numVertices), 1, f);
        fwrite(&numIndices, sizeof(numIndices), 1, f);

        // write data blocks
        fwrite(&m->m_positions[0], sizeof(Vec3) * numVertices, 1, f);
        fwrite(&m->m_normals[0], sizeof(Vec3) * numVertices, 1, f);
        fwrite(&m->m_indices[0], sizeof(int) * numIndices, 1, f);

        fclose(f);
    }
}


// map of Material name to Material
struct VertexKey
{
    VertexKey() : v(0), vt(0), vn(0)
    {
    }

    uint32_t v, vt, vn;

    bool operator==(const VertexKey& rhs) const
    {
        return v == rhs.v && vt == rhs.vt && vn == rhs.vn;
    }

    bool operator<(const VertexKey& rhs) const
    {
        if (v != rhs.v)
            return v < rhs.v;
        else if (vt != rhs.vt)
            return vt < rhs.vt;
        else
            return vn < rhs.vn;
    }
};


void ExportToObj(const char* path, const Mesh& m)
{
    ofstream file(path);

    if (!file)
        return;

    file << "# positions" << endl;

    for (uint32_t i = 0; i < m.m_positions.size(); ++i)
    {
        Point3 v = m.m_positions[i];
        file << "v " << v.x << " " << v.y << " " << v.z << endl;
    }

    file << "# texcoords" << endl;

    for (uint32_t i = 0; i < m.m_texcoords.size(); ++i)
    {
        Vec2 t = m.m_texcoords[0][i];
        file << "vt " << t.x << " " << t.y << endl;
    }

    file << "# normals" << endl;

    for (uint32_t i = 0; i < m.m_normals.size(); ++i)
    {
        Vec3 n = m.m_normals[0][i];
        file << "vn " << n.x << " " << n.y << " " << n.z << endl;
    }

    file << "# faces" << endl;

    for (uint32_t i = 0; i < m.m_indices.size() / 3; ++i)
    {
        // uint32_t j = i+1;

        // no sharing, assumes there is a unique position, texcoord and normal for
        // each vertex
        file << "f " << m.m_indices[i * 3] + 1 << " " << m.m_indices[i * 3 + 1] + 1 << " " << m.m_indices[i * 3 + 2] + 1
             << endl;
    }
}

void Mesh::AddMesh(const Mesh& m)
{
    uint32_t offset = uint32_t(m_positions.size());

    // add new vertices
    m_positions.insert(m_positions.end(), m.m_positions.begin(), m.m_positions.end());
    m_normals.insert(m_normals.end(), m.m_normals.begin(), m.m_normals.end());
    m_colours.insert(m_colours.end(), m.m_colours.begin(), m.m_colours.end());

    // add new indices with offset
    for (uint32_t i = 0; i < m.m_indices.size(); ++i)
    {
        m_indices.push_back(m.m_indices[i] + offset);
    }
}

void Mesh::Flip()
{
    for (int i = 0; i < int(GetNumFaces()); ++i)
    {
        swap(m_indices[i * 3 + 0], m_indices[i * 3 + 1]);
    }

    for (int i = 0; i < (int)m_normals.size(); ++i)
        m_normals[i] *= -1.0f;
}

void Mesh::Transform(const Matrix44& m)
{
    for (uint32_t i = 0; i < m_positions.size(); ++i)
    {
        m_positions[i] = m * m_positions[i];
        m_normals[i] = m * m_normals[i];
    }
}

void Mesh::GetBounds(Vector3& outMinExtents, Vector3& outMaxExtents) const
{
    Point3 minExtents(FLT_MAX);
    Point3 maxExtents(-FLT_MAX);

    // calculate face bounds
    for (uint32_t i = 0; i < m_positions.size(); ++i)
    {
        const Point3& a = m_positions[i];

        minExtents = Min(a, minExtents);
        maxExtents = Max(a, maxExtents);
    }

    outMinExtents = Vector3(minExtents);
    outMaxExtents = Vector3(maxExtents);
}

Mesh* CreateTriMesh(float size, float y)
{
    uint32_t indices[] = { 0, 1, 2 };
    Point3 positions[3];
    Vector3 normals[3];

    positions[0] = Point3(-size, y, size);
    positions[1] = Point3(size, y, size);
    positions[2] = Point3(size, y, -size);

    normals[0] = Vector3(0.0f, 1.0f, 0.0f);
    normals[1] = Vector3(0.0f, 1.0f, 0.0f);
    normals[2] = Vector3(0.0f, 1.0f, 0.0f);

    Mesh* m = new Mesh();
    m->m_indices.insert(m->m_indices.begin(), indices, indices + 3);
    m->m_positions.insert(m->m_positions.begin(), positions, positions + 3);
    m->m_normals.insert(m->m_normals.begin(), normals, normals + 3);

    return m;
}

Mesh* CreateCubeMesh()
{
    const Point3 vertices[24] = { Point3(0.5, 0.5, 0.5),   Point3(-0.5, 0.5, 0.5),   Point3(0.5, -0.5, 0.5),
                                  Point3(-0.5, -0.5, 0.5), Point3(0.5, 0.5, -0.5),   Point3(-0.5, 0.5, -0.5),
                                  Point3(0.5, -0.5, -0.5), Point3(-0.5, -0.5, -0.5), Point3(0.5, 0.5, 0.5),
                                  Point3(0.5, -0.5, 0.5),  Point3(0.5, 0.5, 0.5),    Point3(0.5, 0.5, -0.5),
                                  Point3(-0.5, 0.5, 0.5),  Point3(-0.5, 0.5, -0.5),  Point3(0.5, -0.5, -0.5),
                                  Point3(0.5, 0.5, -0.5),  Point3(-0.5, -0.5, -0.5), Point3(0.5, -0.5, -0.5),
                                  Point3(-0.5, -0.5, 0.5), Point3(0.5, -0.5, 0.5),   Point3(-0.5, -0.5, -0.5),
                                  Point3(-0.5, -0.5, 0.5), Point3(-0.5, 0.5, -0.5),  Point3(-0.5, 0.5, 0.5) };

    const Vec3 normals[24] = { Vec3(0.0f, 0.0f, 1.0f),    Vec3(0.0f, 0.0f, 1.0f),    Vec3(0.0f, 0.0f, 1.0f),
                               Vec3(0.0f, 0.0f, 1.0f),    Vec3(1.0f, 0.0f, 0.0f),    Vec3(0.0f, 1.0f, 0.0f),
                               Vec3(1.0f, 0.0f, 0.0f),    Vec3(0.0f, 0.0f, -1.0f),   Vec3(1.0f, 0.0f, 0.0f),
                               Vec3(1.0f, 0.0f, 0.0f),    Vec3(0.0f, 1.0f, 0.0f),    Vec3(0.0f, 1.0f, 0.0f),
                               Vec3(0.0f, 1.0f, 0.0f),    Vec3(0.0f, 0.0f, -1.0f),   Vec3(0.0f, 0.0f, -1.0f),
                               Vec3(-0.0f, -0.0f, -1.0f), Vec3(0.0f, -1.0f, 0.0f),   Vec3(0.0f, -1.0f, 0.0f),
                               Vec3(0.0f, -1.0f, 0.0f),   Vec3(-0.0f, -1.0f, -0.0f), Vec3(-1.0f, 0.0f, 0.0f),
                               Vec3(-1.0f, 0.0f, 0.0f),   Vec3(-1.0f, 0.0f, 0.0f),   Vec3(-1.0f, -0.0f, -0.0f) };

    const int indices[36] = { 0, 1,  2,  3,  2,  1,  8,  9,  4,  6,  4,  9,  10, 11, 12, 5,  12, 11,
                              7, 13, 14, 15, 14, 13, 16, 17, 18, 19, 18, 17, 20, 21, 22, 23, 22, 21 };

    Mesh* m = new Mesh();
    m->m_positions.assign(vertices, vertices + 24);
    m->m_normals.assign(normals, normals + 24);
    m->m_indices.assign(indices, indices + 36);

    return m;
}

Mesh* CreateQuadMesh(float sizex, float sizez, int gridx, int gridz)
{
    Mesh* m = new Mesh();

    float cellx = sizex / gridz;
    float cellz = sizez / gridz;

    Vec3 start = Vec3(-sizex, 0.0f, sizez) * 0.5f;

    for (int z = 0; z <= gridz; ++z)
    {
        for (int x = 0; x <= gridx; ++x)
        {
            Point3 p = Point3(cellx * x, 0.0f, -cellz * z) + start;

            m->m_positions.push_back(p);
            m->m_normals.push_back(Vec3(0.0f, 1.0f, 0.0f));
            m->m_texcoords.push_back(Vec2(float(x) / gridx, float(z) / gridz));

            if (z > 0 && x > 0)
            {
                int index = int(m->m_positions.size()) - 1;

                m->m_indices.push_back(index);
                m->m_indices.push_back(index - 1);
                m->m_indices.push_back(index - gridx - 1);

                m->m_indices.push_back(index - 1);
                m->m_indices.push_back(index - 1 - gridx - 1);
                m->m_indices.push_back(index - gridx - 1);
            }
        }
    }

    return m;
}

Mesh* CreateDiscMesh(float radius, uint32_t segments)
{
    const uint32_t numVerts = 1 + segments;

    Mesh* m = new Mesh();
    m->m_positions.resize(numVerts);
    m->m_normals.resize(numVerts, Vec3(0.0f, 1.0f, 0.0f));

    m->m_positions[0] = Point3(0.0f);
    m->m_positions[1] = Point3(0.0f, 0.0f, radius);

    for (uint32_t i = 1; i <= segments; ++i)
    {
        uint32_t nextVert = (i + 1) % numVerts;

        if (nextVert == 0)
            nextVert = 1;

        m->m_positions[nextVert] =
            Point3(radius * Sin((float(i) / segments) * k2Pi), 0.0f, radius * Cos((float(i) / segments) * k2Pi));

        m->m_indices.push_back(0);
        m->m_indices.push_back(i);
        m->m_indices.push_back(nextVert);
    }

    return m;
}

Mesh* CreateTetrahedron(float ground, float height)
{
    Mesh* m = new Mesh();

    const float dimValue = 1.0f / sqrtf(2.0f);
    const Point3 vertices[4] = { Point3(-1.0f, ground, -dimValue), Point3(1.0f, ground, -dimValue),
                                 Point3(0.0f, ground + height, dimValue), Point3(0.0f, ground, dimValue) };

    const int indices[12] = { // winding order is counter-clockwise
                              0, 2, 1, 2, 3, 1, 2, 0, 3, 3, 0, 1
    };

    m->m_positions.assign(vertices, vertices + 4);
    m->m_indices.assign(indices, indices + 12);

    m->CalculateNormals();

    return m;
}

Mesh* CreateCylinder(int slices, float radius, float halfHeight, bool cap)
{
    Mesh* mesh = new Mesh();

    for (int i = 0; i <= slices; ++i)
    {
        float theta = (k2Pi / slices) * i;

        Vec3 p = Vec3(sinf(theta), 0.0f, cosf(theta));
        Vec3 n = p;

        mesh->m_positions.push_back(Point3(p * radius - Vec3(0.0f, halfHeight, 0.0f)));
        mesh->m_positions.push_back(Point3(p * radius + Vec3(0.0f, halfHeight, 0.0f)));

        mesh->m_normals.push_back(n);
        mesh->m_normals.push_back(n);

        mesh->m_texcoords.push_back(Vec2(2.0f * float(i) / slices, 0.0f));
        mesh->m_texcoords.push_back(Vec2(2.0f * float(i) / slices, 1.0f));

        if (i > 0)
        {
            int a = (i - 1) * 2 + 0;
            int b = (i - 1) * 2 + 1;
            int c = i * 2 + 0;
            int d = i * 2 + 1;

            // quad between last two vertices and these two
            mesh->m_indices.push_back(a);
            mesh->m_indices.push_back(c);
            mesh->m_indices.push_back(b);

            mesh->m_indices.push_back(c);
            mesh->m_indices.push_back(d);
            mesh->m_indices.push_back(b);
        }
    }
    if (cap)
    {
        // Create cap
        int st = int(mesh->m_positions.size());

        mesh->m_positions.push_back(-Point3(0.0f, halfHeight, 0.0f));
        mesh->m_texcoords.push_back(Vec2(0.0f, 0.0f));
        mesh->m_normals.push_back(-Vec3(0.0f, 1.0f, 0.0f));
        for (int i = 0; i <= slices; ++i)
        {
            float theta = -(k2Pi / slices) * i;

            Vec3 p = Vec3(sinf(theta), 0.0f, cosf(theta));
            mesh->m_positions.push_back(Point3(p * radius - Vec3(0.0f, halfHeight, 0.0f)));
            mesh->m_normals.push_back(-Vec3(0.0f, 1.0f, 0.0f));
            mesh->m_texcoords.push_back(Vec2(2.0f * float(i) / slices, 0.0f));
            if (i > 0)
            {
                mesh->m_indices.push_back(st);
                mesh->m_indices.push_back(st + 1 + i - 1);
                mesh->m_indices.push_back(st + 1 + i % slices);
            }
        }

        st = int(mesh->m_positions.size());
        mesh->m_positions.push_back(Point3(0.0f, halfHeight, 0.0f));
        mesh->m_texcoords.push_back(Vec2(0.0f, 0.0f));
        mesh->m_normals.push_back(Vec3(0.0f, 1.0f, 0.0f));
        for (int i = 0; i <= slices; ++i)
        {
            float theta = (k2Pi / slices) * i;

            Vec3 p = Vec3(sinf(theta), 0.0f, cosf(theta));
            mesh->m_positions.push_back(Point3(p * radius + Vec3(0.0f, halfHeight, 0.0f)));
            mesh->m_normals.push_back(Vec3(0.0f, 1.0f, 0.0f));
            mesh->m_texcoords.push_back(Vec2(2.0f * float(i) / slices, 0.0f));

            if (i > 0)
            {
                mesh->m_indices.push_back(st);
                mesh->m_indices.push_back(st + 1 + i - 1);
                mesh->m_indices.push_back(st + 1 + i % slices);
            }
        }
    }
    return mesh;
}

Mesh* CreateSphere(int slices, int segments, float radius)
{
    float dTheta = kPi / slices;
    float dPhi = k2Pi / segments;

    int vertsPerRow = segments + 1;

    Mesh* mesh = new Mesh();

    for (int i = 0; i <= slices; ++i)
    {
        for (int j = 0; j <= segments; ++j)
        {
            float u = float(i) / slices;
            float v = float(j) / segments;

            float theta = dTheta * i;
            float phi = dPhi * j;

            float x = sinf(theta) * cosf(phi);
            float y = cosf(theta);
            float z = sinf(theta) * sinf(phi);

            mesh->m_positions.push_back(Point3(x, y, z) * radius);
            mesh->m_normals.push_back(Vec3(x, y, z));
            mesh->m_texcoords.push_back(Vec2(u, v));

            if (i > 0 && j > 0)
            {
                int a = i * vertsPerRow + j;
                int b = (i - 1) * vertsPerRow + j;
                int c = (i - 1) * vertsPerRow + j - 1;
                int d = i * vertsPerRow + j - 1;

                // add a quad for this slice
                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(a);
                mesh->m_indices.push_back(d);

                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(d);
                mesh->m_indices.push_back(c);
            }
        }
    }

    return mesh;
}

Mesh* CreateEllipsoid(int slices, int segments, Vec3 radiis)
{
    float dTheta = kPi / slices;
    float dPhi = k2Pi / segments;

    int vertsPerRow = segments + 1;

    Mesh* mesh = new Mesh();

    for (int i = 0; i <= slices; ++i)
    {
        for (int j = 0; j <= segments; ++j)
        {
            float u = float(i) / slices;
            float v = float(j) / segments;

            float theta = dTheta * i;
            float phi = dPhi * j;

            float x = sinf(theta) * cosf(phi);
            float y = cosf(theta);
            float z = sinf(theta) * sinf(phi);

            mesh->m_positions.push_back(Point3(x * radiis.x, y * radiis.y, z * radiis.z));
            mesh->m_normals.push_back(Normalize(Vec3(x / radiis.x, y / radiis.y, z / radiis.z)));
            mesh->m_texcoords.push_back(Vec2(u, v));

            if (i > 0 && j > 0)
            {
                int a = i * vertsPerRow + j;
                int b = (i - 1) * vertsPerRow + j;
                int c = (i - 1) * vertsPerRow + j - 1;
                int d = i * vertsPerRow + j - 1;

                // add a quad for this slice
                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(a);
                mesh->m_indices.push_back(d);

                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(d);
                mesh->m_indices.push_back(c);
            }
        }
    }

    return mesh;
}
Mesh* CreateCapsule(int slices, int segments, float radius, float halfHeight)
{
    float dTheta = kPi / (slices * 2);
    float dPhi = k2Pi / segments;

    int vertsPerRow = segments + 1;

    Mesh* mesh = new Mesh();

    float theta = 0.0f;

    for (int i = 0; i <= 2 * slices + 1; ++i)
    {
        for (int j = 0; j <= segments; ++j)
        {
            float phi = dPhi * j;

            float x = sinf(theta) * cosf(phi);
            float y = cosf(theta);
            float z = sinf(theta) * sinf(phi);

            // add y offset based on which hemisphere we're in
            float yoffset = (i < slices) ? halfHeight : -halfHeight;

            Point3 p = Point3(x, y, z) * radius + Vec3(0.0f, yoffset, 0.0f);

            float u = float(j) / segments;
            float v = (halfHeight + radius + float(p.y)) / fabsf(halfHeight + radius);

            mesh->m_positions.push_back(p);
            mesh->m_normals.push_back(Vec3(x, y, z));
            mesh->m_texcoords.push_back(Vec2(u, v));

            if (i > 0 && j > 0)
            {
                int a = i * vertsPerRow + j;
                int b = (i - 1) * vertsPerRow + j;
                int c = (i - 1) * vertsPerRow + j - 1;
                int d = i * vertsPerRow + j - 1;

                // add a quad for this slice
                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(a);
                mesh->m_indices.push_back(d);

                mesh->m_indices.push_back(b);
                mesh->m_indices.push_back(d);
                mesh->m_indices.push_back(c);
            }
        }

        // don't update theta for the middle slice
        if (i != slices)
            theta += dTheta;
    }

    return mesh;
}
