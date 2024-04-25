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
#include "MjcfTypes.h"
#include "MjcfUtils.h"
#include "math/core/maths.h"

#include <physxSchema/jointStateAPI.h>
#include <physxSchema/physxArticulationAPI.h>
#include <physxSchema/physxJointAPI.h>
#include <physxSchema/physxLimitAPI.h>
#include <physxSchema/physxRigidBodyAPI.h>
#include <physxSchema/physxSceneAPI.h>
#include <physxSchema/physxTendonAttachmentAPI.h>
#include <physxSchema/physxTendonAttachmentLeafAPI.h>
#include <physxSchema/physxTendonAttachmentRootAPI.h>
#include <physxSchema/physxTendonAxisAPI.h>
#include <physxSchema/physxTendonAxisRootAPI.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/filteredPairsAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/limitAPI.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <map>
#include <vector>

namespace omni
{
namespace importer
{
namespace mjcf
{
pxr::SdfPath getNextFreePath(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& primPath);

void setStageMetadata(pxr::UsdStageWeakPtr stage, const omni::importer::mjcf::ImportConfig config);
void createRoot(pxr::UsdStageWeakPtr stage,
                Transform trans,
                const std::string rootPrimPath,
                const omni::importer::mjcf::ImportConfig config);
void createFixedRoot(pxr::UsdStageWeakPtr stage, const std::string jointPath, const std::string bodyPath);
void applyArticulationAPI(pxr::UsdStageWeakPtr stage,
                          pxr::UsdGeomXformable prim,
                          const omni::importer::mjcf::ImportConfig config);
pxr::UsdGeomMesh createMesh(
    pxr::UsdStageWeakPtr stage, const pxr::SdfPath path, Mesh* mesh, float scale, bool importMaterials, bool instanceable);
pxr::UsdGeomMesh createMesh(pxr::UsdStageWeakPtr stage,
                            const pxr::SdfPath path,
                            const std::vector<pxr::GfVec3f>& points,
                            const std::vector<pxr::GfVec3f>& normals,
                            const std::vector<int>& indices,
                            const std::vector<int>& vertexCounts);
void createAndBindMaterial(pxr::UsdStageWeakPtr stage,
                           pxr::UsdPrim prim,
                           MJCFMaterial* material,
                           MJCFTexture* texture,
                           Vec4& color,
                           bool colorOnly);
pxr::UsdGeomXformable createBody(pxr::UsdStageWeakPtr stage,
                                 const std::string primPath,
                                 const Transform& trans,
                                 const ImportConfig& config);
void applyRigidBody(pxr::UsdGeomXformable bodyPrim, const MJCFBody* body, const ImportConfig& config);
pxr::UsdPrim createPrimitiveGeom(pxr::UsdStageWeakPtr stage,
                                 const std::string geomPath,
                                 const MJCFGeom* geom,
                                 const std::map<std::string, MeshInfo>& simulationMeshCache,
                                 const ImportConfig& config,
                                 bool importMaterials,
                                 const std::string rootPrimPath,
                                 bool collisionGeom);
pxr::UsdPrim createPrimitiveGeom(pxr::UsdStageWeakPtr stage,
                                 const std::string geomPath,
                                 const MJCFSite* site,
                                 const ImportConfig& config,
                                 bool importMaterials);
void applyCollisionGeom(pxr::UsdStageWeakPtr stage, pxr::UsdPrim prim, const MJCFGeom* geom);
pxr::UsdPhysicsJoint createFixedJoint(pxr::UsdStageWeakPtr stage,
                                      const std::string jointPath,
                                      const Transform& poseJointToParentBody,
                                      const Transform& poseJointToChildBody,
                                      const std::string parentBodyPath,
                                      const std::string bodyPath,
                                      const ImportConfig& config);
pxr::UsdPhysicsJoint createD6Joint(pxr::UsdStageWeakPtr stage,
                                   const std::string jointPath,
                                   const Transform& poseJointToParentBody,
                                   const Transform& poseJointToChildBody,
                                   const std::string parentBodyPath,
                                   const std::string bodyPath,
                                   const ImportConfig& config);
void initPhysicsJoint(pxr::UsdPhysicsJoint& jointPrim,
                      const Transform& poseJointToParentBody,
                      const Transform& poseJointToChildBody,
                      const std::string parentBodyPath,
                      const std::string bodyPath,
                      const float& distanceScale);
void applyPhysxJoint(pxr::UsdPhysicsJoint& jointPrim, const MJCFJoint* joint);
void applyJointLimits(pxr::UsdPhysicsJoint jointPrim,
                      const MJCFJoint* joint,
                      const MJCFActuator* actuator,
                      const int* axisMap,
                      const int jointIdx,
                      const int numJoints,
                      const ImportConfig& config);
void createJointDrives(pxr::UsdPhysicsJoint jointPrim,
                       const MJCFJoint* joint,
                       const MJCFActuator* actuator,
                       const std::string axis,
                       const ImportConfig& config);

} // namespace mjcf
} // namespace importer
} // namespace omni
