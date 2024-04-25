// SPDX-FileCopyrightText: Copyright (c) 2023-2024, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Mjcf.h"
#include "MjcfParser.h"
#include "MjcfTypes.h"
#include "MjcfUsd.h"
#include "MjcfUtils.h"
#include "core/mesh.h"
#include "math/core/maths.h"

#include <carb/logging/Log.h>

#include <pxr/usd/usdGeom/imageable.h>

#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <tinyxml2.h>
#include <vector>

namespace omni
{
namespace importer
{
namespace mjcf
{

class MJCFImporter
{
public:
    std::string baseDirPath;
    std::string defaultClassName;
    std::map<std::string, MJCFClass> classes;

    MJCFCompiler compiler;
    std::vector<MJCFBody*> bodies;
    std::vector<MJCFGeom*> collisionGeoms;
    std::vector<MJCFActuator*> actuators;
    std::vector<MJCFTendon*> tendons;
    std::vector<MJCFContact*> contacts;
    MJCFBody worldBody;

    std::map<std::string, pxr::UsdPhysicsRevoluteJoint> revoluteJointsMap;
    std::map<std::string, pxr::UsdPhysicsPrismaticJoint> prismaticJointsMap;
    std::map<std::string, pxr::UsdPhysicsJoint> d6JointsMap;

    std::map<std::string, pxr::UsdPrim> geomPrimMap;
    std::map<std::string, pxr::UsdPrim> sitePrimMap;
    std::map<std::string, pxr::UsdPrim> siteToBodyPrim;
    std::map<std::string, pxr::UsdPrim> geomToBodyPrim;

    std::queue<MJCFBody*> bodyQueue;
    std::map<std::string, int> jointToKinematicHierarchy;

    std::map<std::string, int> jointToActuatorIdx;

    std::map<std::string, MeshInfo> simulationMeshCache;
    std::map<std::string, MJCFMesh> meshes;
    std::map<std::string, MJCFMaterial> materials;
    std::map<std::string, MJCFTexture> textures;

    std::vector<ContactNode*> contactGraph;

    std::map<std::string, MJCFBody*> nameToBody;
    std::map<std::string, int> geomNameToIdx;

    std::map<std::string, std::string> nameToUsdCollisionPrim;

    bool createBodyForFixedJoint;

    bool isLoaded = false;

    MJCFImporter(const std::string fullPath, ImportConfig& config);
    ~MJCFImporter();

    void populateBodyLookup(MJCFBody* body);
    bool AddPhysicsEntities(pxr::UsdStageWeakPtr stage,
                            const Transform trans,
                            const std::string& rootPrimPath,
                            const ImportConfig& config);

    void CreateInstanceableMeshes(pxr::UsdStageRefPtr stage,
                                  MJCFBody* body,
                                  const std::string rootPrimPath,
                                  const bool isRoot,
                                  const ImportConfig& config);

    void CreatePhysicsBodyAndJoint(pxr::UsdStageWeakPtr stage,
                                   MJCFBody* body,
                                   std::string rootPrimPath,
                                   const Transform trans,
                                   const bool isRoot,
                                   const std::string parentBodyPath,
                                   const ImportConfig& config,
                                   const std::string instanceableUsdPath);

    void computeJointFrame(Transform& origin, int* axisMap, const MJCFBody* body);

    bool contactBodyExclusion(MJCFBody* body1, MJCFBody* body2);
    bool createContactGraph();

    void computeKinematicHierarchy();

    void addWorldGeomsAndSites(pxr::UsdStageWeakPtr stage,
                               std::string rootPath,
                               const ImportConfig& config,
                               const std::string instanceableUsdPath);
    bool addVisualGeom(pxr::UsdStageWeakPtr stage,
                       pxr::UsdPrim bodyPrim,
                       MJCFBody* body,
                       std::string bodyPath,
                       const ImportConfig& config,
                       bool createGeoms,
                       const std::string rootPrimPath);
    void addVisualSites(pxr::UsdStageWeakPtr stage,
                        pxr::UsdPrim bodyPrim,
                        MJCFBody* body,
                        std::string bodyPath,
                        const ImportConfig& config);

    void AddContactFilters(pxr::UsdStageWeakPtr stage);
    void AddTendons(pxr::UsdStageWeakPtr stage, std::string rootPath);
    pxr::GfVec3f GetLocalPos(MJCFTendon::SpatialAttachment attachment);
};

} // namespace mjcf
} // namespace importer
} // namespace omni
