// Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//

#include "mesh.h"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
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

} // namespace anonymous


static pxr::GfVec3f AiVector3dToGfVector3f(const aiVector3D& vector)
{
    return pxr::GfVec3f(vector.x, vector.y, vector.z);
}

static pxr::GfVec2f AiVector3dToGfVector2f(const aiVector3D& vector)
{
    return pxr::GfVec2f(vector.x, vector.y);
}

pxr::GfVec3f AiColor4DToGfVector3f(const aiColor4D& color)
{
    return pxr::GfVec3f(color.r, color.g, color.b);
}


void addAssimpNodeToMesh(const aiScene* scene, const aiNode* node, aiMatrix4x4 xform, UVInfo& uvInfo, Mesh* mesh)
{
    unsigned int triOffset = static_cast<unsigned int>(mesh->m_indices.size() / 3);
    unsigned int pointOffset = static_cast<unsigned int>(mesh->m_positions.size());
    unsigned int nodeTriOffset = 0;
    unsigned int nodePointOffset = 0;

    for (unsigned int m = 0; m < node->mNumMeshes; ++m)
    {
        const aiMesh* assimpMesh = scene->mMeshes[node->mMeshes[m]];
        USDMesh usdmesh;

        for (unsigned int j = 0; j < assimpMesh->mNumVertices; ++j)
        {
            const aiVector3D& p = xform * assimpMesh->mVertices[j];
            mesh->m_positions.push_back(Point3{ p.x, p.y, p.z });
            usdmesh.points.push_back(AiVector3dToGfVector3f(p));
        }

        unsigned int numColourChannels = assimpMesh->GetNumColorChannels();
        usdmesh.colors.resize(numColourChannels);
        if (numColourChannels > 0)
        {
            if (numColourChannels > 1)
            {
                std::cout << "Multiple colour channels not supported. Using first channel." << std::endl;
            }
            unsigned int colourChannel = 0;
            for (; colourChannel < AI_MAX_NUMBER_OF_COLOR_SETS; ++colourChannel)
            {
                if (assimpMesh->HasVertexColors(colourChannel))
                    break;
            }
            for (unsigned int j = 0; j < assimpMesh->mNumVertices; ++j)
            {
                const aiColor4D& c = assimpMesh->mColors[colourChannel][j];
                mesh->m_colours.push_back(Colour{ c.r, c.g, c.b, c.a });
            }
        }

        unsigned int numUVChannels = assimpMesh->GetNumUVChannels();
        usdmesh.uvs.resize(numUVChannels);
        if (numUVChannels > 0)
        {
            if (numUVChannels > 1)
            {
                std::cout << "Multiple UV channels not supported. Using first channel." << std::endl;
            }
            unsigned int UVChannel = 0;
            for (; UVChannel < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++UVChannel)
            {
                if (assimpMesh->HasTextureCoords(UVChannel) && assimpMesh->mNumUVComponents[UVChannel] <= 2)
                    break;
            }

            uvInfo.uvs.emplace_back();
            auto& currentUV = uvInfo.uvs.back();
            uvInfo.uvStartIndices.push_back(pointOffset + nodePointOffset);

            for (unsigned int j = 0; j < assimpMesh->mNumVertices; ++j)
            {
                const aiVector3D& uv = assimpMesh->mTextureCoords[UVChannel][j];
                mesh->m_texcoords.push_back(Vector2{ uv.x, uv.y });

                currentUV.push_back(Vector2{ uv.x, uv.y });
            }
        }

        for (size_t j = 0; j < assimpMesh->mNumFaces; j++)
        {
            const aiFace& face = assimpMesh->mFaces[j];
            if (face.mNumIndices >= 3)
            {
                for (size_t k = 0; k < face.mNumIndices; k++)
                {
                    if (assimpMesh->mNormals)
                    {
                        usdmesh.normals.push_back(AiVector3dToGfVector3f(assimpMesh->mNormals[face.mIndices[k]]));
                    }

                    for (size_t m = 0; m < usdmesh.uvs.size(); m++)
                    {
                        usdmesh.uvs[m].push_back(AiVector3dToGfVector2f(assimpMesh->mTextureCoords[m][face.mIndices[k]]));
                    }

                    for (size_t m = 0; m < usdmesh.colors.size(); m++)
                    {
                        usdmesh.colors[m].push_back(AiColor4DToGfVector3f(assimpMesh->mColors[m][face.mIndices[k]]));
                    }
                }
                usdmesh.faceVertexCounts.push_back(face.mNumIndices);
            }
        }

        unsigned int indexOffset = pointOffset + nodePointOffset;

        for (unsigned int j = 0; j < assimpMesh->mNumFaces; ++j)
        {
            const aiFace& f = assimpMesh->mFaces[j];
            if (f.mNumIndices >= 3)
            {
                for (size_t k = 0; k < f.mNumIndices; k++)
                {
                    usdmesh.faceVertexIndices.push_back(f.mIndices[k]);
                }
            }
            // assert(f.mNumIndices > 0 && f.mNumIndices <= 3); // importer should triangluate mesh
            if (f.mNumIndices == 1)
            {
                mesh->m_indices.push_back(f.mIndices[0] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[0] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[0] + indexOffset);
            }
            else if (f.mNumIndices == 2)
            {
                mesh->m_indices.push_back(f.mIndices[0] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[1] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[1] + indexOffset);
            }
            else if (f.mNumIndices == 3)
            {
                mesh->m_indices.push_back(f.mIndices[0] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[1] + indexOffset);
                mesh->m_indices.push_back(f.mIndices[2] + indexOffset);
            }
        }

        if (assimpMesh->HasNormals())
        {
            for (unsigned int j = 0; j < assimpMesh->mNumVertices; ++j)
            {
                const aiVector3D& n = xform * assimpMesh->mNormals[j];
                mesh->m_normals.push_back(SafeNormalize(Vector3{ n.x, n.y, n.z }));
            }
        }

        MaterialAssignment matAssign;
        matAssign.startTri = triOffset + nodeTriOffset;
        matAssign.endTri = triOffset + nodeTriOffset + assimpMesh->mNumFaces;
        matAssign.material = static_cast<int>(assimpMesh->mMaterialIndex);
        mesh->m_materialAssignments.push_back(matAssign);

        nodeTriOffset += assimpMesh->mNumFaces;
        nodePointOffset += assimpMesh->mNumVertices;

        mesh->m_usdMeshPrims.push_back(usdmesh);
    }
}


Mesh* ImportMeshAssimp(const char* path)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(std::string(path), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

    if (!scene)
    {
        return nullptr;
    }

    Mesh* mesh = new Mesh;
    for (unsigned int i = 0; i < scene->mNumMaterials; ++i)
    {
        aiMaterial* assimpMaterial = scene->mMaterials[i];
        Material mat;
        mat.name = std::string{ assimpMaterial->GetName().C_Str() };

        aiColor3D Ka;
        if (assimpMaterial->Get(AI_MATKEY_COLOR_AMBIENT, Ka) == AI_SUCCESS)
        {
            mat.Ka = Vec3{ Ka.r, Ka.g, Ka.b };
        }

        aiColor3D Kd;
        if (assimpMaterial->Get(AI_MATKEY_COLOR_DIFFUSE, Kd) == AI_SUCCESS)
        {
            mat.Kd = Vec3{ Kd.r, Kd.g, Kd.b };
            mat.hasDiffuse = true;
        }

        aiColor3D Ks;
        if (assimpMaterial->Get(AI_MATKEY_COLOR_SPECULAR, Ks) == AI_SUCCESS)
        {
            mat.Ks = Vec3{ Ks.r, Ks.g, Ks.b };
        }

        float specular;
        if (assimpMaterial->Get(AI_MATKEY_SPECULAR_FACTOR, specular) == AI_SUCCESS)
        {
            mat.specular = specular;
            mat.hasSpecular = true;
        }

        float Ns;
        if (assimpMaterial->Get(AI_MATKEY_SHININESS, Ns) == AI_SUCCESS)
        {
            mat.Ns = Ns;
            mat.hasShininess = true;
        }

        float metallic;
        if (assimpMaterial->Get(AI_MATKEY_METALLIC_FACTOR, metallic) == AI_SUCCESS)
        {
            mat.metallic = metallic;
            mat.hasMetallic = true;
        }

        aiColor3D emissive;
        if (assimpMaterial->Get(AI_MATKEY_COLOR_EMISSIVE, emissive) == AI_SUCCESS)
        {
            mat.emissive = Vec3{ emissive.r, emissive.g, emissive.b };
            mat.hasEmissive = true;
        }

        aiString path;
        if (assimpMaterial->GetTexture(aiTextureType_DIFFUSE, 0, &path) == aiReturn_SUCCESS)
        {
            mat.mapKd = std::string(path.C_Str());
        }
        if (assimpMaterial->GetTexture(aiTextureType_HEIGHT, 0, &path) == aiReturn_SUCCESS)
        {
            mat.mapBump = std::string(path.C_Str());
        }
        if (assimpMaterial->GetTexture(aiTextureType_REFLECTION, 0, &path) == aiReturn_SUCCESS)
        {
            mat.mapMetallic = std::string(path.C_Str());
        }
        if (assimpMaterial->GetTexture(aiTextureType_EMISSIVE, 0, &path) == aiReturn_SUCCESS)
        {
            mat.mapEnv = std::string(path.C_Str());
        }
        if (assimpMaterial->GetTexture(aiTextureType_SHININESS, 0, &path) == aiReturn_SUCCESS)
        {
            mat.mapKs = std::string(path.C_Str());
        }

        mesh->m_materials.push_back(mat);
    }

    UVInfo uvInfo;
    std::vector<std::pair<const aiNode*, aiMatrix4x4>> nodeStack;

    const aiNode* root = scene->mRootNode;
    nodeStack.push_back(std::make_pair(root, root->mTransformation));

    while (!nodeStack.empty())
    {
        auto nodeAndTransform = nodeStack.back();
        const aiNode* node = nodeAndTransform.first;
        aiMatrix4x4 xform = nodeAndTransform.second;

        nodeStack.pop_back();

        addAssimpNodeToMesh(scene, node, xform, uvInfo, mesh);

        for (unsigned int c = 0; c < node->mNumChildren; ++c)
        {
            const aiNode* child = node->mChildren[c];
            nodeStack.push_back(std::make_pair(child, xform * child->mTransformation));
        }
    }

    return mesh;
}


Mesh* ImportMesh(const char* path)
{
    std::string ext = GetExtension(path);

    Mesh* mesh = nullptr;

    if (ext == "ply")
        mesh = ImportMeshFromPly(path);
    else if (ext == "obj")
        mesh = ImportMeshFromObj(path);
    else if (ext == "stl")
        mesh = ImportMeshFromStl(path);


    return mesh;
}

Mesh* ImportMeshFromBin(const char* path)
{
    // double start = GetSeconds();

    FILE* f = fopen(path, "rb");

    if (f)
    {
        int numVertices;
        int numIndices;

        size_t len;
        len = fread(&numVertices, sizeof(numVertices), 1, f);
        len = fread(&numIndices, sizeof(numIndices), 1, f);

        Mesh* m = new Mesh();
        m->m_positions.resize(numVertices);
        m->m_normals.resize(numVertices);
        m->m_indices.resize(numIndices);

        len = fread(&m->m_positions[0], sizeof(Vec3) * numVertices, 1, f);
        len = fread(&m->m_normals[0], sizeof(Vec3) * numVertices, 1, f);
        len = fread(&m->m_indices[0], sizeof(int) * numIndices, 1, f);

        (void)len;

        fclose(f);

        // double end = GetSeconds();

        //    printf("Imported mesh %s in %f ms\n", path, (end - start) * 1000.0f);

        return m;
    }

    return nullptr;
}

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

Mesh* ImportMeshFromPly(const char* path)
{
    ifstream file(path, ios_base::in | ios_base::binary);

    if (!file)
        return nullptr;

    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    // double startTime = GetSeconds();

    file >> buffer;
    if (strcmp(buffer, "ply") != 0)
        return nullptr;

    PlyFormat format = eAscii;

    uint32_t numFaces = 0;
    uint32_t numVertices = 0;

    const uint32_t kMaxProperties = 16;
    uint32_t numProperties = 0;
    float properties[kMaxProperties];

    bool vertexElement = false;

    while (file)
    {
        file >> buffer;

        if (strcmp(buffer, "element") == 0)
        {
            file >> buffer;

            if (strcmp(buffer, "face") == 0)
            {
                vertexElement = false;
                file >> numFaces;
            }

            else if (strcmp(buffer, "vertex") == 0)
            {
                vertexElement = true;
                file >> numVertices;
            }
        }
        else if (strcmp(buffer, "format") == 0)
        {
            file >> buffer;
            if (strcmp(buffer, "ascii") == 0)
            {
                format = eAscii;
            }
            else if (strcmp(buffer, "binary_big_endian") == 0)
            {
                format = eBinaryBigEndian;
            }
            else
            {
                printf("Ply: unknown format\n");
                return nullptr;
            }
        }
        else if (strcmp(buffer, "property") == 0)
        {
            if (vertexElement)
                ++numProperties;
        }
        else if (strcmp(buffer, "end_header") == 0)
        {
            break;
        }
    }

    // eat newline
    char nl;
    file.read(&nl, 1);

    // debug
#if ENABLE_VERBOSE_OUTPUT
    printf("Loaded mesh: %s numFaces: %d numVertices: %d format: %d numProperties: %d\n", path, numFaces, numVertices,
           format, numProperties);
#endif

    Mesh* mesh = new Mesh;

    mesh->m_positions.resize(numVertices);
    mesh->m_normals.resize(numVertices);
    mesh->m_colours.resize(numVertices, Colour(1.0f, 1.0f, 1.0f, 1.0f));

    mesh->m_indices.reserve(numFaces * 3);

    // read vertices
    for (uint32_t v = 0; v < numVertices; ++v)
    {
        for (uint32_t i = 0; i < numProperties; ++i)
        {
            properties[i] = PlyRead<float>(file, format);
        }

        mesh->m_positions[v] = Point3(properties[0], properties[1], properties[2]);
        mesh->m_normals[v] = Vector3(0.0f, 0.0f, 0.0f);
    }

    // read indices
    for (uint32_t f = 0; f < numFaces; ++f)
    {
        uint32_t numIndices = (format == eAscii) ? PlyRead<uint32_t>(file, format) : PlyRead<uint8_t>(file, format);
        uint32_t indices[4];

        for (uint32_t i = 0; i < numIndices; ++i)
        {
            indices[i] = PlyRead<uint32_t>(file, format);
        }

        switch (numIndices)
        {
        case 3:
            mesh->m_indices.push_back(indices[0]);
            mesh->m_indices.push_back(indices[1]);
            mesh->m_indices.push_back(indices[2]);
            break;
        case 4:
            mesh->m_indices.push_back(indices[0]);
            mesh->m_indices.push_back(indices[1]);
            mesh->m_indices.push_back(indices[2]);

            mesh->m_indices.push_back(indices[2]);
            mesh->m_indices.push_back(indices[3]);
            mesh->m_indices.push_back(indices[0]);
            break;

        default:
            assert(!"invalid number of indices, only support tris and quads");
            break;
        };

        // calculate vertex normals as we go
        Point3& v0 = mesh->m_positions[indices[0]];
        Point3& v1 = mesh->m_positions[indices[1]];
        Point3& v2 = mesh->m_positions[indices[2]];

        Vector3 n = SafeNormalize(Cross(v1 - v0, v2 - v0), Vector3(0.0f, 1.0f, 0.0f));

        for (uint32_t i = 0; i < numIndices; ++i)
        {
            mesh->m_normals[indices[i]] += n;
        }
    }

    for (uint32_t i = 0; i < numVertices; ++i)
    {
        mesh->m_normals[i] = SafeNormalize(mesh->m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }

    // cout << "Imported mesh " << path << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;

    return mesh;
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


void ImportFromMtlLib(const char* path, std::vector<Material>& materials)
{
    FILE* f = fopen(path, "r");

    const int kMaxLineLength = 1024;

    if (f)
    {
        char line[kMaxLineLength];

        while (fgets(line, kMaxLineLength, f))
        {
            char name[kMaxLineLength];

            if (sscanf(line, " newmtl %s", name) == 1)
            {
                Material mat;
                mat.name = name;

                materials.push_back(mat);
            }

            if (materials.size())
            {
                Material& mat = materials.back();

                sscanf(line, " Ka %f %f %f", &mat.Ka.x, &mat.Ka.y, &mat.Ka.z);
                sscanf(line, " Kd %f %f %f", &mat.Kd.x, &mat.Kd.y, &mat.Kd.z);
                sscanf(line, " Ks %f %f %f", &mat.Ks.x, &mat.Ks.y, &mat.Ks.z);
                sscanf(line, " Ns %f", &mat.Ns);
                sscanf(line, " metallic %f", &mat.metallic);

                char map[kMaxLineLength];
                if (sscanf(line, " map_Kd %s", map) == 1)
                    mat.mapKd = map;

                if (sscanf(line, " map_Ks %s", map) == 1)
                    mat.mapKs = map;

                if (sscanf(line, " map_bump %s", map) == 1)
                    mat.mapBump = map;

                if (sscanf(line, " map_env %s", map) == 1)
                    mat.mapEnv = map;
            }
        }

        fclose(f);
    }
}


Mesh* ImportMeshFromObj(const char* meshPath)
{
    ifstream file(meshPath);

    if (!file)
        return nullptr;

    Mesh* m = new Mesh();

    vector<Point3> positions;
    vector<Vector3> normals;
    vector<Vector2> texcoords;
    vector<Vector3> colors;
    vector<uint32_t>& indices = m->m_indices;

    // typedef unordered_map<VertexKey, uint32_t, MemoryHash<VertexKey> > VertexMap;
    typedef map<VertexKey, uint32_t> VertexMap;
    VertexMap vertexLookup;

    // some scratch memory
    const uint32_t kMaxLineLength = 1024;
    char buffer[kMaxLineLength];

    // double startTime = GetSeconds();

    file >> buffer;
    while (!file.eof())
    {
        if (strcmp(buffer, "vn") == 0)
        {
            // normals
            float x, y, z;
            file >> x >> y >> z;

            normals.push_back(Vector3(x, y, z));
        }
        else if (strcmp(buffer, "vt") == 0)
        {
            // texture coords
            float u, v;
            file >> u >> v;

            texcoords.push_back(Vector2(u, v));
        }
        else if (buffer[0] == 'v')
        {
            // positions
            float x, y, z;
            file >> x >> y >> z;

            positions.push_back(Point3(x, y, z));
        }
        else if (buffer[0] == 's' || buffer[0] == 'g' || buffer[0] == 'o')
        {
            // ignore smoothing groups, groups and objects
            char linebuf[256];
            file.getline(linebuf, 256);
        }
        else if (strcmp(buffer, "mtllib") == 0)
        {
            std::string materialFile;
            file >> materialFile;

            char materialPath[2048];
            MakeRelativePath(meshPath, materialFile.c_str(), materialPath);

            ImportFromMtlLib(materialPath, m->m_materials);
        }
        else if (strcmp(buffer, "usemtl") == 0)
        {
            // read Material name, ignored right now
            std::string materialName;
            file >> materialName;

            // if there was a previous assignment then close it
            if (m->m_materialAssignments.size())
                m->m_materialAssignments.back().endTri = int(indices.size() / 3);

            // generate assignment
            MaterialAssignment batch;
            batch.startTri = int(indices.size() / 3);
            batch.material = -1;

            for (int i = 0; i < (int)m->m_materials.size(); ++i)
            {
                if (m->m_materials[i].name == materialName)
                {
                    batch.material = i;
                    break;
                }
            }

            if (batch.material == -1)
                printf(".obj references material not found in .mtl library, %s\n", materialName.c_str());
            else
            {
                // push back assignment
                m->m_materialAssignments.push_back(batch);
            }
        }
        else if (buffer[0] == 'f')
        {
            // faces
            uint32_t faceIndices[4];
            uint32_t faceIndexCount = 0;

            for (int i = 0; i < 4; ++i)
            {
                VertexKey key;
                file >> key.v;

                // failed to read another index continue on
                if (file.fail())
                {
                    file.clear();
                    break;
                }

                if (file.peek() == '/')
                {
                    file.ignore();

                    if (file.peek() != '/')
                    {
                        file >> key.vt;
                    }

                    if (file.peek() == '/')
                    {
                        file.ignore();
                        file >> key.vn;
                    }
                }

                // find / add vertex, index
                VertexMap::iterator iter = vertexLookup.find(key);

                if (iter != vertexLookup.end())
                {
                    faceIndices[faceIndexCount++] = iter->second;
                }
                else
                {
                    // add vertex
                    uint32_t newIndex = uint32_t(m->m_positions.size());
                    faceIndices[faceIndexCount++] = newIndex;

                    vertexLookup.insert(make_pair(key, newIndex));

                    // push back vertex data
                    assert(key.v > 0);

                    m->m_positions.push_back(positions[key.v - 1]);

                    // obj format doesn't support mesh colours so add default value
                    m->m_colours.push_back(Colour(1.0f, 1.0f, 1.0f));

                    // normal
                    if (key.vn)
                    {
                        m->m_normals.push_back(normals[key.vn - 1]);
                    }
                    else
                    {
                        m->m_normals.push_back(0.0f);
                    }

                    // texcoord
                    if (key.vt)
                    {
                        m->m_texcoords.push_back(texcoords[key.vt - 1]);
                    }
                    else
                    {
                        m->m_texcoords.push_back(0.0f);
                    }
                }
            }

            if (faceIndexCount < 3)
            {
                cout << "File contains face(s) with less than 3 vertices" << endl;
            }
            else if (faceIndexCount == 3)
            {
                // a triangle
                indices.insert(indices.end(), faceIndices, faceIndices + 3);
            }
            else if (faceIndexCount == 4)
            {
                // a quad, triangulate clockwise
                indices.insert(indices.end(), faceIndices, faceIndices + 3);

                indices.push_back(faceIndices[2]);
                indices.push_back(faceIndices[3]);
                indices.push_back(faceIndices[0]);
            }
            else
            {
                cout << "Face with more than 4 vertices are not supported" << endl;
            }
        }
        else if (buffer[0] == '#')
        {
            // comment
            char linebuf[256];
            file.getline(linebuf, 256);
        }

        file >> buffer;
    }

    // calculate normals if none specified in file
    m->m_normals.resize(m->m_positions.size());

    const uint32_t numFaces = uint32_t(indices.size()) / 3;
    for (uint32_t i = 0; i < numFaces; ++i)
    {
        uint32_t a = indices[i * 3 + 0];
        uint32_t b = indices[i * 3 + 1];
        uint32_t c = indices[i * 3 + 2];

        Point3& v0 = m->m_positions[a];
        Point3& v1 = m->m_positions[b];
        Point3& v2 = m->m_positions[c];

        Vector3 n = SafeNormalize(Cross(v1 - v0, v2 - v0), Vector3(0.0f, 1.0f, 0.0f));

        m->m_normals[a] += n;
        m->m_normals[b] += n;
        m->m_normals[c] += n;
    }

    for (uint32_t i = 0; i < m->m_normals.size(); ++i)
    {
        m->m_normals[i] = SafeNormalize(m->m_normals[i], Vector3(0.0f, 1.0f, 0.0f));
    }

    // close final material assignment
    if (m->m_materialAssignments.size())
        m->m_materialAssignments.back().endTri = int(indices.size()) / 3;

    // cout << "Imported mesh " << meshPath << " in " << (GetSeconds()-startTime)*1000.f << "ms" << endl;

    return m;
}

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

        // no sharing, assumes there is a unique position, texcoord and normal for each vertex
        file << "f " << m.m_indices[i * 3] + 1 << " " << m.m_indices[i * 3 + 1] + 1 << " " << m.m_indices[i * 3 + 2] + 1
             << endl;
    }
}

Mesh* ImportMeshFromStl(const char* path)
{
    // double start = GetSeconds();

    FILE* f = fopen(path, "rb");

    if (f)
    {

        char header[80];
        fread(header, 80, 1, f);

        int numTriangles;
        fread(&numTriangles, sizeof(int), 1, f);

        Mesh* m = new Mesh();
        m->m_positions.resize(numTriangles * 3);
        m->m_normals.resize(numTriangles * 3);
        m->m_indices.resize(numTriangles * 3);

        Point3* vertexPtr = m->m_positions.data();
        Vector3* normalPtr = m->m_normals.data();
        uint32_t* indexPtr = m->m_indices.data();

        for (int t = 0; t < numTriangles; ++t)
        {
            Vector3 n;
            Point3 v0, v1, v2;
            uint16_t attributeByteCount;

            fread(&n, sizeof(Vector3), 1, f);
            fread(&v0, sizeof(Point3), 1, f);
            fread(&v1, sizeof(Point3), 1, f);
            fread(&v2, sizeof(Point3), 1, f);
            fread(&attributeByteCount, sizeof(uint16_t), 1, f);

            *(normalPtr++) = n;
            *(normalPtr++) = n;
            *(normalPtr++) = n;
            *(vertexPtr++) = v0;
            *(vertexPtr++) = v1;
            *(vertexPtr++) = v2;
            *(indexPtr++) = t * 3 + 0;
            *(indexPtr++) = t * 3 + 1;
            *(indexPtr++) = t * 3 + 2;
        }

        fclose(f);

        // double end = GetSeconds();

        // printf("Imported mesh %s in %f ms\n", path, (end - start) * 1000.0f);

        return m;
    }

    return nullptr;
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
