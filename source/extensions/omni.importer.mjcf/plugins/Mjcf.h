// Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
//
// NVIDIA CORPORATION and its licensors retain all intellectual property
// and proprietary rights in and to this software, related documentation
// and any modifications thereto. Any use, reproduction, disclosure or
// distribution of this software and related documentation without an express
// license agreement from NVIDIA CORPORATION is strictly prohibited.
//
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

}
}
}
