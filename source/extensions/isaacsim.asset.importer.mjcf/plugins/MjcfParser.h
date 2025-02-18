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

#include "Mjcf.h"
#include "MjcfTypes.h"
#include "math/core/maths.h"

#include <map>
#include <tinyxml2.h>

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{

tinyxml2::XMLElement* LoadInclude(tinyxml2::XMLDocument& doc, const tinyxml2::XMLElement* c, const std::string baseDirPath);
void LoadCompiler(tinyxml2::XMLElement* c, MJCFCompiler& compiler);
void LoadInertial(tinyxml2::XMLElement* i, MJCFInertial& inertial);
void LoadGeom(tinyxml2::XMLElement* g,
              MJCFGeom& geom,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              bool isDefault);
void LoadSite(tinyxml2::XMLElement* s,
              MJCFSite& site,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              bool isDefault);
void LoadMesh(tinyxml2::XMLElement* m,
              MJCFMesh& mesh,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes);
void LoadActuator(tinyxml2::XMLElement* g,
                  MJCFActuator& actuator,
                  std::string className,
                  MJCFActuator::Type type,
                  std::map<std::string, MJCFClass>& classes);
void LoadContact(tinyxml2::XMLElement* g,
                 MJCFContact& contact,
                 MJCFContact::Type type,
                 std::map<std::string, MJCFClass>& classes);
void LoadTendon(tinyxml2::XMLElement* t,
                MJCFTendon& tendon,
                std::string className,
                MJCFTendon::Type type,
                std::map<std::string, MJCFClass>& classes);
void LoadJoint(tinyxml2::XMLElement* g,
               MJCFJoint& joint,
               std::string className,
               MJCFCompiler& compiler,
               std::map<std::string, MJCFClass>& classes,
               bool isDefault);
void LoadFreeJoint(tinyxml2::XMLElement* g,
                   MJCFJoint& joint,
                   std::string className,
                   MJCFCompiler& compiler,
                   std::map<std::string, MJCFClass>& classes,
                   bool isDefault);
void LoadDefault(tinyxml2::XMLElement* e,
                 const std::string className,
                 MJCFClass& cl,
                 MJCFCompiler& compiler,
                 std::map<std::string, MJCFClass>& classes);
void LoadBody(tinyxml2::XMLElement* g,
              std::vector<MJCFBody*>& bodies,
              MJCFBody& body,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              std::string baseDirPath);
tinyxml2::XMLElement* LoadFile(tinyxml2::XMLDocument& doc, const std::string filePath);
void LoadAssets(tinyxml2::XMLElement* a,
                std::string baseDirPath,
                MJCFCompiler& compiler,
                std::map<std::string, MeshInfo>& simulationMeshCache,
                std::map<std::string, MJCFMesh>& meshes,
                std::map<std::string, MJCFMaterial>& materials,
                std::map<std::string, MJCFTexture>& textures,
                std::string className,
                std::map<std::string, MJCFClass>& classes,
                ImportConfig& config);
void LoadGlobals(tinyxml2::XMLElement* root,
                 std::string& defaultClassName,
                 std::string baseDirPath,
                 MJCFBody& worldBody,
                 std::vector<MJCFBody*>& bodies,
                 std::vector<MJCFActuator*>& actuators,
                 std::vector<MJCFTendon*>& tendons,
                 std::vector<MJCFContact*>& contacts,
                 std::vector<MJCFEqualityConnect*>& equalityConnects,
                 std::map<std::string, MeshInfo>& simulationMeshCache,
                 std::map<std::string, MJCFMesh>& meshes,
                 std::map<std::string, MJCFMaterial>& materials,
                 std::map<std::string, MJCFTexture>& textures,
                 MJCFCompiler& compiler,
                 std::map<std::string, MJCFClass>& classes,
                 std::map<std::string, int>& jointToActuatorIdx,
                 ImportConfig& config);

} // namespace mjcf
} // namespace importer
} // namespace asset
} // namespace isaacsim
