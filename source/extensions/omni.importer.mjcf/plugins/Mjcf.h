// SPDX-FileCopyrightText: Copyright (c) 2022-2024, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/Defines.h>

#include <pybind11/pybind11.h>

#include <stdint.h>

namespace omni
{
namespace importer
{
namespace mjcf
{

struct ImportConfig
{
    bool mergeFixedJoints = false;
    bool convexDecomp = false;
    bool importInertiaTensor = false;
    bool fixBase = true;
    bool selfCollision = false;
    float density = 1000; // default density used for objects without mass/inertia
    // UrdfJointTargetType defaultDriveType = UrdfJointTargetType::POSITION;
    float defaultDriveStrength = 100000;
    float distanceScale = 1.0f;
    // UrdfAxis upVector = { 0, 0, 1 };
    bool createPhysicsScene = true;
    bool makeDefaultPrim = true;

    bool createBodyForFixedJoint = true;
    bool overrideCoM = false;
    bool overrideInertia = false;
    bool visualizeCollisionGeoms = false;
    bool importSites = true;

    bool makeInstanceable = false;
    std::string instanceableMeshUsdPath = "./instanceable_meshes.usd";
};

struct Mjcf
{
    CARB_PLUGIN_INTERFACE("omni::importer::mjcf::Mjcf", 0, 1);

    void(CARB_ABI* createAssetFromMJCF)(const char* fileName,
                                        const char* primName,
                                        ImportConfig& config,
                                        const std::string& stage_identifier);
};

} // namespace mjcf
} // namespace importer
} // namespace omni
