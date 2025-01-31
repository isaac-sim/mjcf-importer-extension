// SPDX-FileCopyrightText: Copyright (c) 2023-2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include "UsdPCH.h"
// clang-format on

#include "core/mesh.h"
#include "omniverse_asset_converter.h"
#include "utils/Path.h"
#include "utils/Usd.h"

#include <fstream>
using namespace isaacsim::asset::utils::path;
namespace mesh
{

inline std::string StatusToString(OmniConverterStatus status)
{
    switch (status)
    {
    case OmniConverterStatus::OK:
        return "OK";
    case OmniConverterStatus::CANCELLED:
        return "Cancelled";
    case OmniConverterStatus::IN_PROGRESS:
        return "In Progress";
    case OmniConverterStatus::UNSUPPORTED_IMPORT_FORMAT:
        return "Unsupported Format";
    case OmniConverterStatus::INCOMPLETE_IMPORT_FORMAT:
        return "Incomplete File";
    case OmniConverterStatus::FILE_READ_ERROR:
        return "Asset Not Found";
    case OmniConverterStatus::FILE_WRITE_ERROR:
        return "Output Path Cannot be Opened";
    case OmniConverterStatus::UNKNOWN:
        return "Unknown";
    default:
        return "Unknown";
    }
}


class MeshImporter
{
private:
    // std::map<std::string, std::pair<Mesh*, carb::gym::GymMeshHandle>>
    // gymGraphicsMeshCache; std::map<std::pair<float, float>,
    // carb::gym::TriangleMeshHandle> cylinderCache; std::map<std::string, Mesh*>
    // simulationMeshCache;

public:
    MeshImporter()
    {
    }

    std::string resolveMeshPath(const std::string& filePath)
    {
        // noop for now
        return filePath;
    }

    pxr::TfToken getMaterialToken(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& materialPath)
    {
        // Get the material object from the stage
        pxr::UsdShadeMaterial material = pxr::UsdShadeMaterial(stage->GetPrimAtPath(materialPath));

        // If the material doesn't exist, return an empty token
        if (!material)
        {
            return pxr::TfToken();
        }

        auto shaderPrim = *material.GetPrim().GetChildren().begin();
        // Get the shader object for the material
        pxr::UsdShadeShader shader(shaderPrim);
        if (!shader)
        {
            return pxr::TfToken();
        }

        std::stringstream ss;
        for (const auto& attr : shaderPrim.GetAttributes())
        {
            ss << "<";
            ss << attr.GetTypeName() << " : " << attr.GetName() << " = ";

            pxr::VtValue value;
            if (attr.Get(&value))
            {
                ss << value << ">\n";
            }
            else
            {
                ss << ">\n"; // Handle cases where the attribute has no value
            }
        }

        // Create a TfToken from the token string
        pxr::TfToken token(ss.str());
        return token;
    }


    void moveAndBindMaterial(const pxr::UsdStageRefPtr& sourceStage,
                             const pxr::UsdStageRefPtr& dstStage,
                             const pxr::SdfPath& rootPath,
                             const pxr::SdfPath& sourcePath,
                             const pxr::SdfPath& dstPath,
                             std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        auto sourceImageable = sourceStage->GetPrimAtPath(sourcePath);
        auto dstImageable = dstStage->GetPrimAtPath(dstPath);
        pxr::UsdShadeMaterialBindingAPI bindingAPI(sourceImageable);
        pxr::UsdShadeMaterialBindingAPI::DirectBinding directBinding = bindingAPI.GetDirectBinding();
        pxr::UsdShadeMaterial material = directBinding.GetMaterial();
        if (material)
        {
            pxr::TfToken materialToken = getMaterialToken(sourceStage, material.GetPrim().GetPath());
            pxr::SdfPath materialPath;
            if (materialPaths.count(materialToken))
            {
                materialPath = materialPaths[materialToken];
            }
            else
            {
                materialPath = pxr::SdfPath(isaacsim::asset::importer::mjcf::usd::GetNewSdfPathString(
                    dstStage, rootPath.AppendChild(pxr::TfToken("Looks"))
                                  .AppendChild(pxr::TfToken(material.GetPath().GetName()))
                                  .GetString()));
                pxr::SdfCopySpec(sourceStage->GetRootLayer(), material.GetPath(), dstStage->GetRootLayer(), materialPath);
                materialPaths.emplace(materialToken, materialPath);
            }

            pxr::UsdShadeMaterial newMaterial = pxr::UsdShadeMaterial(dstStage->GetPrimAtPath(materialPath));
            if (newMaterial)
            {
                bindingAPI = pxr::UsdShadeMaterialBindingAPI(dstImageable);
                bindingAPI.Bind(newMaterial);
            }
        }
    }

    void moveMeshAndMaterials(const pxr::UsdStageRefPtr& sourceStage,
                              const pxr::UsdStageRefPtr& dstStage,
                              const pxr::SdfPath& rootPath,
                              const pxr::SdfPath& meshPath,
                              const pxr::SdfPath& targetPrimPath,
                              std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh(sourceStage->GetPrimAtPath(meshPath));
        if (!mesh)
        {
            CARB_LOG_ERROR("Error: Could not find mesh at path %s", meshPath.GetText());
            return;
        }

        pxr::UsdGeomMesh newMesh = pxr::UsdGeomMesh::Define(dstStage, targetPrimPath);
        if (!newMesh)
        {
            CARB_LOG_ERROR("Error: Could not create new mesh at path %s", targetPrimPath.GetText());
            return;
        }

        pxr::SdfCopySpec(sourceStage->GetRootLayer(), mesh.GetPath(), dstStage->GetRootLayer(), newMesh.GetPath());
        // Move materials and bind them to the new mesh
        for (const auto& subset : pxr::UsdGeomSubset::GetAllGeomSubsets(mesh))
        {
            pxr::TfToken subsetName = subset.GetPrim().GetName();
            pxr::UsdGeomSubset dstSubset =
                pxr::UsdGeomSubset(dstStage->GetPrimAtPath(newMesh.GetPath().AppendChild(pxr::TfToken(subsetName))));
            moveAndBindMaterial(sourceStage, dstStage, rootPath, subset.GetPath(), dstSubset.GetPath(), materialPaths);
        }

        // Bind the material to the new mesh
        moveAndBindMaterial(sourceStage, dstStage, rootPath, mesh.GetPath(), newMesh.GetPath(), materialPaths);
    }


    pxr::SdfPath waitForConverter(OmniConverterFuture* future,
                                  pxr::UsdStageRefPtr usdStage,
                                  const std::string& mesh_usd_path,
                                  const std::string& meshStagePath,
                                  const pxr::SdfPath& rootPath,
                                  std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        OmniConverterStatus status = OmniConverterStatus::OK;
        // Wait for the converter to finish
        if (future == nullptr)
        {
            CARB_LOG_ERROR("Error: Future is null");
            return pxr::SdfPath();
        }
        while (omniConverterCheckFutureStatus(future) == OmniConverterStatus::IN_PROGRESS)
        {
        }
        status = omniConverterCheckFutureStatus(future);
        omniConverterReleaseFuture(future);

        if (status == OmniConverterStatus::OK)
        {
            CARB_LOG_INFO("Asset %s convert successfully.", mesh_usd_path.c_str());
        }
        else
        {
            CARB_LOG_WARN("Asset convert failed with error status: %s (%s)", StatusToString(status).c_str(),
                          meshStagePath.c_str());
        }
        // Open the mesh stage and get the mesh prims
        pxr::UsdStageRefPtr meshStage = pxr::UsdStage::Open(mesh_usd_path);
        pxr::UsdPrimRange primRange(meshStage->GetDefaultPrim());
        std::vector<pxr::UsdPrim> meshPrims;
        std::copy_if(primRange.begin(), primRange.end(), std::back_inserter(meshPrims),
                     [](pxr::UsdPrim const& prim) { return prim.IsA<pxr::UsdGeomMesh>(); });


        // Create a base prim and move meshes and materials
        auto basePrim = pxr::UsdGeomXform::Define(usdStage, pxr::SdfPath(meshStagePath));

        for (const auto& mesh : meshPrims)
        {
            moveMeshAndMaterials(meshStage, usdStage, usdStage->GetDefaultPrim().GetPath(), mesh.GetPath(),
                                 basePrim.GetPath().AppendChild(pxr::TfToken(mesh.GetName())), materialPaths);
        }
        // Clean up
        // omniClientWait(omniClientDelete(mesh_usd_path.c_str(), {}, {}));
        pxr::UsdGeomImageable(usdStage->GetPrimAtPath(pxr::SdfPath(meshStagePath)))
            .CreateVisibilityAttr()
            .Set(pxr::UsdGeomTokens->inherited);
        return pxr::SdfPath(meshStagePath);
    }

    void importMesh(Mesh* mesh, std::string relativeMeshPath, const Vec3& scale, bool flip = false)
    {
        std::string meshPath = resolveMeshPath(relativeMeshPath);
        std::string mesh_usd_path = pathJoin(getParent(meshPath), getPathStem(meshPath.c_str()) + ".tmp.usd");

        CARB_LOG_INFO("Importing Mesh %s\n    (%s)", meshPath.c_str(), mesh_usd_path.c_str());
        // Set up the Omni Converter flags
        int flags = OMNI_CONVERTER_FLAGS_SINGLE_MESH_FILE | OMNI_CONVERTER_FLAGS_IGNORE_CAMERAS |
                    OMNI_CONVERTER_FLAGS_USE_METER_PER_UNIT | OMNI_CONVERTER_FLAGS_MERGE_ALL_MESHES |
                    OMNI_CONVERTER_FLAGS_IGNORE_LIGHTS | OMNI_CONVERTER_FLAGS_FBX_CONVERT_TO_Z_UP |
                    OMNI_CONVERTER_FLAGS_FBX_BAKING_SCALES_INTO_MESH | OMNI_CONVERTER_FLAGS_IGNORE_PIVOTS;

        // Set up the log and progress callbacks for the Omni Converter
        omniConverterSetLogCallback([](const char* message) { CARB_LOG_INFO(message); });
        omniConverterSetProgressCallback([](OmniConverterFuture* future, uint32_t progress, uint32_t total)
                                         { CARB_LOG_INFO("Progress: %d / %d", progress, total); });

        // Create a new Omni Converter future for the mesh import
        mesh->m_assetConvertStatus = omniConverterCreateAsset(meshPath.c_str(), mesh_usd_path.c_str(), flags);
        mesh->m_convertedUsdMesh = mesh_usd_path;
    }
};

} // namespace mesh
