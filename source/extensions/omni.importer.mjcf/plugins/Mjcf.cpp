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

#define CARB_EXPORTS

// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "Mjcf.h"

#include "MjcfImporter.h"
#include "stdio.h"

#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>

#include <omni/ext/IExt.h>
#include <omni/kit/IApp.h>
#include <omni/kit/IStageUpdate.h>

#define EXTENSION_NAME "omni.importer.mjcf.plugin"

using namespace carb;

const struct carb::PluginImplDesc kPluginImpl = { EXTENSION_NAME, "MJCF Utilities", "NVIDIA",
                                                  carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::importer::mjcf::Mjcf)
CARB_PLUGIN_IMPL_DEPS(omni::kit::IApp, carb::logging::ILogging)

namespace
{

// passed in from python
void createAssetFromMJCF(const char* fileName,
                         const char* primName,
                         omni::importer::mjcf::ImportConfig& config,
                         const std::string& stage_identifier = "")
{
    omni::importer::mjcf::MJCFImporter mjcf(fileName, config);
    if (!mjcf.isLoaded)
    {
        printf("cannot load mjcf xml file\n");
    }
    Transform trans = Transform();

    bool save_stage = true;
    pxr::UsdStageRefPtr _stage;
    if (stage_identifier != "" && pxr::UsdStage::IsSupportedFile(stage_identifier))
    {
        _stage = pxr::UsdStage::Open(stage_identifier);
        if (!_stage)
        {
            CARB_LOG_INFO("Creating Stage: %s", stage_identifier.c_str());
            _stage = pxr::UsdStage::CreateNew(stage_identifier);
        }
        else
        {
            for (const auto& p : _stage->GetPrimAtPath(pxr::SdfPath("/")).GetChildren())
            {
                _stage->RemovePrim(p.GetPath());
            }
        }
        config.makeDefaultPrim = true;
        pxr::UsdGeomSetStageUpAxis(_stage, pxr::UsdGeomTokens->z);
    }
    if (!_stage) // If all else fails, import on current stage
    {
        CARB_LOG_INFO("Importing URDF to Current Stage");
        // Get the 'active' USD stage from the USD stage cache.
        const std::vector<pxr::UsdStageRefPtr> allStages = pxr::UsdUtilsStageCache::Get().GetAllStages();
        if (allStages.size() != 1)
        {
            CARB_LOG_ERROR(
                "Cannot determine the 'active' USD stage (%zu stages "
                "present in the USD stage cache).",
                allStages.size());
            return;
        }

        _stage = allStages[0];
        save_stage = false;
    }
    std::string result = "";
    if (_stage)
    {
        pxr::UsdGeomSetStageMetersPerUnit(_stage, 1.0 / config.distanceScale);
        if (!mjcf.AddPhysicsEntities(_stage, trans, primName, config))
        {
            printf("no physics entities found!\n");
        }
        // CARB_LOG_WARN("Import Done, saving");
        if (save_stage)
        {
            // CARB_LOG_WARN("Saving Stage %s",
            // _stage->GetRootLayer()->GetIdentifier().c_str());
            _stage->Save();
        }
    }
}

} // namespace

CARB_EXPORT void carbOnPluginStartup()
{
    CARB_LOG_INFO("Startup MJCF Extension");
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

void fillInterface(omni::importer::mjcf::Mjcf& iface)
{
    using namespace omni::importer::mjcf;
    memset(&iface, 0, sizeof(iface));
    // iface.helloWorld = helloWorld;
    iface.createAssetFromMJCF = createAssetFromMJCF;
    // iface.importRobot = importRobot;
}
