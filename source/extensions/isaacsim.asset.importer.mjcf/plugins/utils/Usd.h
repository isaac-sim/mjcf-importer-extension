// SPDX-FileCopyrightText: Copyright (c) 2024-2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <pxr/pxr.h>
#include <pxr/usd/usd/stage.h>
// clang-format on

/**
 * Select a existing layer as edit target.
 *
 * @param stage The stage of the operation.
 * @param layerIdentifier Layer identifier.
 * @return true if the layer is selected, false otherwise.
 *
 **/
namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{
namespace usd
{

// Make a path name that is not already used.
inline std::string GetNewSdfPathString(pxr::UsdStageWeakPtr stage, std::string path, int nameClashNum = 0)
{
    bool appendedNumber = false;
    int numberAppended = std::max<int>(nameClashNum, 0);
    size_t indexOfNumber = 0;
    if (stage->GetPrimAtPath(pxr::SdfPath(path)))
    {
        appendedNumber = true;
        std::string name = pxr::SdfPath(path).GetName();
        size_t last_ = name.find_last_of('_');
        indexOfNumber = path.length() + 1;
        if (last_ == std::string::npos)
        {
            // no '_' found, so just tack on the end.
            path += "_" + std::to_string(numberAppended);
        }
        else
        {
            // There was a _, if the last part of that is a number
            // then replace that number with one higher or nameClashNum,
            // or just tack on the number if it is last character.
            if (last_ == name.length() - 1)
            {
                path += "_" + std::to_string(numberAppended);
            }
            else
            {
                char* p;
                std::string after_ = name.substr(last_ + 1, name.length());
                long converted = strtol(after_.c_str(), &p, 10);
                if (*p)
                {
                    // not a number
                    path += "_" + std::to_string(numberAppended);
                }
                else
                {

                    numberAppended = nameClashNum == -1 ? converted + 1 : nameClashNum;
                    indexOfNumber = path.length() - name.length() + last_ + 1;
                    path = path.substr(0, indexOfNumber);
                    path += std::to_string(numberAppended);
                }
            }
        }
    }
    if (appendedNumber)
    {
        // we just added a number, so we have to make sure the new path is unique.
        while (stage->GetPrimAtPath(pxr::SdfPath(path)))
        {
            path = path.substr(0, indexOfNumber);
            numberAppended += 1;
            path += std::to_string(numberAppended);
        }
    }
#if 0
	else
	{
		while (stage->GetPrimAtPath(pxr::SdfPath(path))) path += ":" + std::to_string(nameClashNum);
	}
#endif
    return path;
}

} // namespace usd
} // namespace mjcf
} // namespace importer
} // namespace asset
} // namespace isaacsim
