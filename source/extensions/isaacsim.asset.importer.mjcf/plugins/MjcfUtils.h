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

#include "math/core/maths.h"

#include <tinyxml2.h>

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{

std::string SanitizeUsdName(const std::string& src);
std::string GetAttr(const tinyxml2::XMLElement* c, const char* name);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, bool& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, int& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, float& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, std::string& s);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, Vec2& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, Vec3& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, Vec3& from, Vec3& to);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, Vec4& p);
void getIfExist(tinyxml2::XMLElement* e, const char* aname, Quat& q);
void getEulerIfExist(tinyxml2::XMLElement* e, const char* aname, Quat& q, std::string eulerseq, bool angleInRad);
void getZAxisIfExist(tinyxml2::XMLElement* e, const char* aname, Quat& q);
void getAngleAxisIfExist(tinyxml2::XMLElement* e, const char* aname, Quat& q, bool angleInRad);
Quat indexedRotation(int axis, float s, float c);
Vec3 Diagonalize(const Matrix33& m, Quat& massFrame);

} // namespace mjcf
} // namespace importer
} // namespace asset
} // namespace isaacsim
