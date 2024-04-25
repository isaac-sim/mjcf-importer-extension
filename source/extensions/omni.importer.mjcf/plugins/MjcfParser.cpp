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

#include "MjcfParser.h"

#include "MeshImporter.h"
#include "MjcfUtils.h"

#include <carb/logging/Log.h>

namespace omni
{
namespace importer
{
namespace mjcf
{

int bodyIdxCount = 0;
int geomIdxCount = 0;
int siteIdxCount = 0;
int jointIdxCount = 0;

tinyxml2::XMLElement* LoadInclude(tinyxml2::XMLDocument& doc, const tinyxml2::XMLElement* c, const std::string baseDirPath)
{
    if (c)
    {
        std::string s;
        if ((s = GetAttr(c, "file")) != "")
        {
            std::string fileName(s);
            std::string filePath = baseDirPath + fileName;

            tinyxml2::XMLElement* root = LoadFile(doc, filePath);
            return root;
        }
    }
    return nullptr;
}

void LoadCompiler(tinyxml2::XMLElement* c, MJCFCompiler& compiler)
{
    if (c)
    {
        std::string s;

        if ((s = GetAttr(c, "eulerseq")) != "")
        {
            for (int i = (int)s.length() - 1; i >= 0; i--)
            {
                char axis = s[i];
                if (axis == 'X' || axis == 'Y' || axis == 'Z')
                {
                    CARB_LOG_ERROR(
                        "The MJCF importer currently only supports intrinsic "
                        "euler rotations!");
                }
            }
            compiler.eulerseq = s;
        }

        if ((s = GetAttr(c, "angle")) != "")
        {
            compiler.angleInRad = (s == "radian");
        }

        if ((s = GetAttr(c, "inertiafromgeom")) != "")
        {
            compiler.inertiafromgeom = (s == "true");
        }

        if ((s = GetAttr(c, "coordinate")) != "")
        {
            compiler.coordinateInLocal = (s == "local");
            if (!compiler.coordinateInLocal)
            {
                CARB_LOG_ERROR("The global coordinate is no longer supported by MuJoCo!");
            }
        }

        getIfExist(c, "meshdir", compiler.meshDir);
        getIfExist(c, "texturedir", compiler.textureDir);
        getIfExist(c, "autolimits", compiler.autolimits);
    }
}

void LoadInertial(tinyxml2::XMLElement* i, MJCFInertial& inertial)
{
    if (!i)
    {
        return;
    }
    getIfExist(i, "mass", inertial.mass);
    getIfExist(i, "pos", inertial.pos);
    getIfExist(i, "diaginertia", inertial.diaginertia);

    float fullInertia[6];
    const char* st = i->Attribute("fullinertia");
    if (st)
    {
        sscanf(st, "%f %f %f %f %f %f", &fullInertia[0], &fullInertia[1], &fullInertia[2], &fullInertia[3],
               &fullInertia[4], &fullInertia[5]);
        inertial.hasFullInertia = true;

        Matrix33 inertiaMatrix;
        inertiaMatrix.cols[0] = Vec3(fullInertia[0], fullInertia[3], fullInertia[4]);
        inertiaMatrix.cols[1] = Vec3(fullInertia[3], fullInertia[1], fullInertia[5]);
        inertiaMatrix.cols[2] = Vec3(fullInertia[4], fullInertia[5], fullInertia[2]);

        Quat principalAxes;
        inertial.diaginertia = Diagonalize(inertiaMatrix, principalAxes);
        inertial.principalAxes = principalAxes;
    }
}

void LoadGeom(tinyxml2::XMLElement* g,
              MJCFGeom& geom,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              bool isDefault)
{
    if (!g)
    {
        return;
    }
    if (g->Attribute("class"))
        className = g->Attribute("class");
    geom = classes[className].dgeom;

    getIfExist(g, "conaffinity", geom.conaffinity);
    getIfExist(g, "condim", geom.condim);
    getIfExist(g, "contype", geom.contype);
    getIfExist(g, "margin", geom.margin);
    getIfExist(g, "friction", geom.friction);
    getIfExist(g, "material", geom.material);
    getIfExist(g, "rgba", geom.rgba);
    getIfExist(g, "solimp", geom.solimp);
    getIfExist(g, "solref", geom.solref);
    getIfExist(g, "fromto", geom.from, geom.to);
    getIfExist(g, "size", geom.size);
    getIfExist(g, "name", geom.name);
    getIfExist(g, "pos", geom.pos);
    getEulerIfExist(g, "euler", geom.quat, compiler.eulerseq, compiler.angleInRad);
    getAngleAxisIfExist(g, "axisangle", geom.quat, compiler.angleInRad);
    getZAxisIfExist(g, "zaxis", geom.quat);
    getIfExist(g, "quat", geom.quat);
    getIfExist(g, "density", geom.density);
    getIfExist(g, "mesh", geom.mesh);

    if (geom.name == "" && !isDefault)
    {
        geom.name = "_geom_" + std::to_string(geomIdxCount);
        geomIdxCount++;
    }

    if (g->Attribute("fromto"))
    {
        geom.hasFromTo = true;
    }

    std::string type = "";
    getIfExist(g, "type", type);
    if (type == "capsule")
    {
        geom.type = MJCFGeom::CAPSULE;
    }
    else if (type == "sphere")
    {
        geom.type = MJCFGeom::SPHERE;
    }
    else if (type == "ellipsoid")
    {
        geom.type = MJCFGeom::ELLIPSOID;
    }
    else if (type == "cylinder")
    {
        geom.type = MJCFGeom::CYLINDER;
    }
    else if (type == "box")
    {
        geom.type = MJCFGeom::BOX;
    }
    else if (type == "mesh")
    {
        geom.type = MJCFGeom::MESH;
    }
    else if (type == "plane")
    {
        geom.type = MJCFGeom::PLANE;
    }
    else if (type != "")
    {
        geom.type = MJCFGeom::OTHER;
        std::cout << "Geom type " << type << " not yet supported!" << std::endl;
    }

    if (!isDefault && geom.name == "")
    {
        geom.name = type;
    }
}

void LoadSite(tinyxml2::XMLElement* s,
              MJCFSite& site,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              bool isDefault)
{
    if (!s)
    {
        return;
    }
    if (s->Attribute("class"))
        className = s->Attribute("class");
    site = classes[className].dsite;

    getIfExist(s, "material", site.material);
    getIfExist(s, "rgba", site.rgba);
    getIfExist(s, "fromto", site.from, site.to);
    getIfExist(s, "size", site.size);
    getIfExist(s, "name", site.name);
    getIfExist(s, "pos", site.pos);
    getEulerIfExist(s, "euler", site.quat, compiler.eulerseq, compiler.angleInRad);
    getAngleAxisIfExist(s, "axisangle", site.quat, compiler.angleInRad);
    getZAxisIfExist(s, "zaxis", site.quat);
    getIfExist(s, "quat", site.quat);

    if (site.name == "" && !isDefault)
    {
        site.name = "_site_" + std::to_string(siteIdxCount);
        siteIdxCount++;
    }

    if (s->Attribute("fromto") || classes[className].dsite.hasFromTo)
    {
        site.hasFromTo = true;
    }

    if ((!s->Attribute("size") && !classes[className].dsite.hasGeom) && !site.hasFromTo)
    {
        site.hasGeom = false;
    }

    std::string type = "";
    getIfExist(s, "type", type);
    if (type == "capsule")
    {
        site.type = MJCFSite::CAPSULE;
    }
    else if (type == "sphere")
    {
        site.type = MJCFSite::SPHERE;
    }
    else if (type == "ellipsoid")
    {
        site.type = MJCFSite::ELLIPSOID;
    }
    else if (type == "cylinder")
    {
        site.type = MJCFSite::CYLINDER;
    }
    else if (type == "box")
    {
        site.type = MJCFSite::BOX;
    }
    else if (type != "")
    {
        std::cout << "Site type " << type << " not yet supported!" << std::endl;
    }

    if (!isDefault && site.name == "")
    {
        site.name = type;
    }
}

void LoadMesh(tinyxml2::XMLElement* m,
              MJCFMesh& mesh,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes)
{
    if (!m)
    {
        return;
    }
    if (m->Attribute("class"))
        className = m->Attribute("class");
    mesh = classes[className].dmesh;

    getIfExist(m, "name", mesh.name);
    getIfExist(m, "file", mesh.filename);
    getIfExist(m, "scale", mesh.scale);
}

void LoadActuator(tinyxml2::XMLElement* g,
                  MJCFActuator& actuator,
                  std::string className,
                  MJCFActuator::Type type,
                  std::map<std::string, MJCFClass>& classes)
{
    if (!g)
    {
        return;
    }

    if (g->Attribute("class"))
    {
        className = g->Attribute("class");
    }
    actuator = classes[className].dactuator;
    actuator.type = type;
    getIfExist(g, "ctrllimited", actuator.ctrllimited);
    getIfExist(g, "forcelimited", actuator.forcelimited);
    getIfExist(g, "ctrlrange", actuator.ctrlrange);
    getIfExist(g, "forcerange", actuator.forcerange);
    getIfExist(g, "gear", actuator.gear);
    getIfExist(g, "joint", actuator.joint);
    getIfExist(g, "name", actuator.name);

    // actuator specific attributes
    getIfExist(g, "kp", actuator.kp);
    getIfExist(g, "kv", actuator.kv);
}

void LoadContact(tinyxml2::XMLElement* g,
                 MJCFContact& contact,
                 MJCFContact::Type type,
                 std::map<std::string, MJCFClass>& classes)
{
    if (!g)
    {
        return;
    }

    getIfExist(g, "name", contact.name);
    if (type == MJCFContact::PAIR)
    {
        getIfExist(g, "geom1", contact.geom1);
        getIfExist(g, "geom2", contact.geom2);
        getIfExist(g, "condim", contact.condim);
    }
    else if (type == MJCFContact::EXCLUDE)
    {
        getIfExist(g, "body1", contact.body1);
        getIfExist(g, "body2", contact.body2);
    }
    contact.type = type;
}

void LoadTendon(tinyxml2::XMLElement* t,
                MJCFTendon& tendon,
                std::string className,
                MJCFTendon::Type type,
                std::map<std::string, MJCFClass>& classes)
{
    if (!t)
    {
        return;
    }
    if (t->Attribute("class"))
        className = t->Attribute("class");
    tendon = classes[className].dtendon;

    tendon.type = type;

    // parse tendon parameters:
    getIfExist(t, "name", tendon.name);
    getIfExist(t, "limited", tendon.limited);
    getIfExist(t, "range", tendon.range);
    getIfExist(t, "solimplimit", tendon.solimplimit);
    getIfExist(t, "solreflimit", tendon.solreflimit);
    getIfExist(t, "solimpfriction", tendon.solimpfriction);
    getIfExist(t, "solreffriction", tendon.solreffriction);
    getIfExist(t, "margin", tendon.margin);
    getIfExist(t, "frictionloss", tendon.frictionloss);
    getIfExist(t, "width", tendon.width);
    getIfExist(t, "material", tendon.material);
    getIfExist(t, "rgba", tendon.rgba);
    getIfExist(t, "springlength", tendon.springlength);
    if (tendon.springlength < 0.0f)
    {
        CARB_LOG_WARN(
            "*** Automatic tendon springlength calculation is not "
            "supported (negative springlengths).");
    }
    getIfExist(t, "stiffness", tendon.stiffness);
    getIfExist(t, "damping", tendon.damping);

    // and then go through the joints in the fixed tendon:
    if (type == MJCFTendon::FIXED)
    {
        tinyxml2::XMLElement* j = t->FirstChildElement("joint");
        while (j)
        {
            // parse fixed joint:
            if (!j->Attribute("joint"))
            {
                CARB_LOG_FATAL("*** Fixed tendon joint must have a joint attribute.");
            }
            if (!j->Attribute("coef"))
            {
                CARB_LOG_FATAL("*** Fixed tendon joint must have a coef attribute.");
            }
            MJCFTendon::FixedJoint* jnt = new MJCFTendon::FixedJoint();
            getIfExist(j, "joint", jnt->joint);
            getIfExist(j, "coef", jnt->coef);

            // if coef nonzero, add:
            if (0.0f != jnt->coef)
            {
                tendon.fixedJoints.push_back(jnt);
            }

            // scan for next joint in tendon:
            j = j->NextSiblingElement("joint");
        }
    }

    // attributes for spatial teondon
    if (type == MJCFTendon::SPATIAL)
    {
        tinyxml2::XMLElement* x = t->FirstChildElement();
        while (x)
        {
            int branch = 0;
            if (std::string(x->Value()).compare("geom") == 0)
            {
                MJCFTendon::SpatialAttachment* attachment = new MJCFTendon::SpatialAttachment();
                attachment->type = MJCFTendon::SpatialAttachment::GEOM;
                getIfExist(x, "geom", attachment->geom);
                getIfExist(x, "sidesite", attachment->sidesite);
                attachment->branch = branch;
                if (attachment->geom != "")
                {
                    tendon.spatialAttachments.push_back(attachment);
                    tendon.spatialBranches[branch].push_back(attachment);
                }
                else
                    CARB_LOG_FATAL("*** Spatial tendon geom must be specified.");
            }
            else if (std::string(x->Value()).compare("site") == 0)
            {
                MJCFTendon::SpatialAttachment* attachment = new MJCFTendon::SpatialAttachment();
                attachment->type = MJCFTendon::SpatialAttachment::SITE;
                getIfExist(x, "site", attachment->site);
                attachment->branch = branch;
                if (attachment->site != "")
                {
                    tendon.spatialAttachments.push_back(attachment);
                    tendon.spatialBranches[branch].push_back(attachment);
                }
                else
                    CARB_LOG_FATAL("*** Spatial tendon site must be specified.");
            }
            else if (std::string(x->Value()).compare("pulley") == 0)
            {
                MJCFTendon::SpatialPulley* pulley = new MJCFTendon::SpatialPulley();
                getIfExist(x, "divisor", pulley->divisor);
                if (pulley->divisor > 0.0)
                {
                    branch++;
                    pulley->branch = branch;
                    tendon.spatialPulleys.push_back(pulley);
                }
                else
                    CARB_LOG_FATAL("*** Spatial tendon pulley divisor must be specified.");
            }
            else
            {
                CARB_LOG_WARN("Found unknown tag %s in tendon.\n", x->Value());
            }

            x = x->NextSiblingElement();
        }
    }
}

void LoadJoint(tinyxml2::XMLElement* g,
               MJCFJoint& joint,
               std::string className,
               MJCFCompiler& compiler,
               std::map<std::string, MJCFClass>& classes,
               bool isDefault)
{
    if (!g)
    {
        return;
    }
    if (g->Attribute("class"))
        className = g->Attribute("class");
    joint = classes[className].djoint;

    std::string type = "";
    getIfExist(g, "type", type);
    if (type == "hinge")
    {
        joint.type = MJCFJoint::HINGE;
    }
    else if (type == "slide")
    {
        joint.type = MJCFJoint::SLIDE;
    }
    else if (type == "ball")
    {
        joint.type = MJCFJoint::BALL;
    }
    else if (type == "free")
    {
        joint.type = MJCFJoint::FREE;
    }
    else if (type != "")
    {
        std::cout << "JointSpec type " << type << " not yet supported!" << std::endl;
    }
    getIfExist(g, "ref", joint.ref);
    getIfExist(g, "armature", joint.armature);
    getIfExist(g, "damping", joint.damping);
    getIfExist(g, "limited", joint.limited);
    getIfExist(g, "axis", joint.axis);
    getIfExist(g, "name", joint.name);
    getIfExist(g, "pos", joint.pos);
    getIfExist(g, "range", joint.range);

    const char* st = g->Attribute("range");
    if (st)
    {
        sscanf(st, "%f %f", &joint.range.x, &joint.range.y);
        if (compiler.autolimits)
        {
            // set limited to true if a range is specified and autolimits is set to
            // true
            joint.limited = true;
        }
    }

    if (joint.type != MJCFJoint::Type::SLIDE && !compiler.angleInRad)
    {
        // cout << "Angle in deg" << endl;
        joint.range.x = kPi * joint.range.x / 180.0f;
        joint.range.y = kPi * joint.range.y / 180.0f;
    }
    getIfExist(g, "stiffness", joint.stiffness);
    joint.axis = Normalize(joint.axis);

    if (joint.name == "" && !isDefault)
    {
        joint.name = "_joint_" + std::to_string(jointIdxCount);
        jointIdxCount++;
    }
}

void LoadFreeJoint(tinyxml2::XMLElement* g,
                   MJCFJoint& joint,
                   std::string className,
                   MJCFCompiler& compiler,
                   std::map<std::string, MJCFClass>& classes,
                   bool isDefault)
{
    if (!g)
    {
        return;
    }
    if (g->Attribute("class"))
        className = g->Attribute("class");
    joint = classes[className].djoint;

    joint.type = MJCFJoint::FREE;

    getIfExist(g, "name", joint.name);
    if (joint.name == "" && !isDefault)
    {
        joint.name = "_joint_" + std::to_string(jointIdxCount);
        jointIdxCount++;
    }
}

void LoadDefault(tinyxml2::XMLElement* e,
                 const std::string className,
                 MJCFClass& cl,
                 MJCFCompiler& compiler,
                 std::map<std::string, MJCFClass>& classes)
{
    LoadJoint(e->FirstChildElement("joint"), cl.djoint, className, compiler, classes, true);
    LoadGeom(e->FirstChildElement("geom"), cl.dgeom, className, compiler, classes, true);
    LoadSite(e->FirstChildElement("site"), cl.dsite, className, compiler, classes, true);
    LoadTendon(e->FirstChildElement("tendon"), cl.dtendon, className, MJCFTendon::DEFAULT, classes);
    LoadMesh(e->FirstChildElement("mesh"), cl.dmesh, className, compiler, classes);

    // a defaults class should have one general actuator element, so only one of
    // these should be sucessful
    LoadActuator(e->FirstChildElement("motor"), cl.dactuator, className, MJCFActuator::MOTOR, classes);
    LoadActuator(e->FirstChildElement("position"), cl.dactuator, className, MJCFActuator::POSITION, classes);
    LoadActuator(e->FirstChildElement("velocity"), cl.dactuator, className, MJCFActuator::VELOCITY, classes);
    LoadActuator(e->FirstChildElement("general"), cl.dactuator, className, MJCFActuator::GENERAL, classes);

    tinyxml2::XMLElement* d = e->FirstChildElement("default");
    // while there is child default
    while (d)
    {
        // must have a name
        if (!d->Attribute("class"))
        {
            CARB_LOG_ERROR("Non-top level class must have name");
        }

        std::string name = d->Attribute("class");
        classes[name] = cl; // Copy from this class
        LoadDefault(d, name, classes[name], compiler,
                    classes); // Recursively load it
        d = d->NextSiblingElement("default");
    }
}

void LoadBody(tinyxml2::XMLElement* g,
              std::vector<MJCFBody*>& bodies,
              MJCFBody& body,
              std::string className,
              MJCFCompiler& compiler,
              std::map<std::string, MJCFClass>& classes,
              std::string baseDirPath)
{
    if (!g)
    {
        return;
    }

    if (g->Attribute("childclass"))
    {
        className = g->Attribute("childclass");
    }
    getIfExist(g, "name", body.name);
    getIfExist(g, "pos", body.pos);
    getEulerIfExist(g, "euler", body.quat, compiler.eulerseq, compiler.angleInRad);
    getAngleAxisIfExist(g, "axisangle", body.quat, compiler.angleInRad);
    getZAxisIfExist(g, "zaxis", body.quat);
    getIfExist(g, "quat", body.quat);

    if (body.name == "")
    {
        body.name = "_body_" + std::to_string(bodyIdxCount);
        bodyIdxCount++;
    }

    // load interial
    tinyxml2::XMLElement* c = g->FirstChildElement("inertial");
    if (c)
    {
        body.inertial = new MJCFInertial();
        LoadInertial(c, *body.inertial);
    }

    // load geoms
    c = g->FirstChildElement("geom");
    while (c)
    {
        body.geoms.push_back(new MJCFGeom());
        LoadGeom(c, *body.geoms.back(), className, compiler, classes, false);
        c = c->NextSiblingElement("geom");
    }

    // load sites
    c = g->FirstChildElement("site");
    while (c)
    {
        body.sites.push_back(new MJCFSite());
        LoadSite(c, *body.sites.back(), className, compiler, classes, false);
        c = c->NextSiblingElement("site");
    }

    // load joints
    c = g->FirstChildElement("joint");
    while (c)
    {
        body.joints.push_back(new MJCFJoint());
        LoadJoint(c, *body.joints.back(), className, compiler, classes, false);
        c = c->NextSiblingElement("joint");
    }

    // load freejoint
    c = g->FirstChildElement("freejoint");
    if (c)
    {
        body.joints.push_back(new MJCFJoint());
        LoadFreeJoint(c, *body.joints.back(), className, compiler, classes, false);
    }

    // load imports
    c = g->FirstChildElement("include");
    while (c)
    {
        tinyxml2::XMLDocument includeDoc;
        tinyxml2::XMLElement* includeRoot = LoadInclude(includeDoc, c, baseDirPath);
        if (includeRoot)
        {
            tinyxml2::XMLElement* d = includeRoot->FirstChildElement("body");
            while (d)
            {
                bodies.push_back(new MJCFBody());
                LoadBody(d, bodies, *bodies.back(), className, compiler, classes, baseDirPath);
                d = d->NextSiblingElement("body");
            }
        }
        c = c->NextSiblingElement("include");
    }

    // load child bodies
    c = g->FirstChildElement("body");
    while (c)
    {
        body.bodies.push_back(new MJCFBody());
        LoadBody(c, bodies, *body.bodies.back(), className, compiler, classes, baseDirPath);
        c = c->NextSiblingElement("body");
    }
}

tinyxml2::XMLElement* LoadFile(tinyxml2::XMLDocument& doc, const std::string filePath)
{
    if (doc.LoadFile(filePath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        CARB_LOG_ERROR("*** Failed to load '%s'", filePath.c_str());
        return nullptr;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root)
    {
        CARB_LOG_ERROR("*** Empty document '%s'", filePath.c_str());
    }

    return root;
}

void LoadAssets(tinyxml2::XMLElement* a,
                std::string baseDirPath,
                MJCFCompiler& compiler,
                std::map<std::string, MeshInfo>& simulationMeshCache,
                std::map<std::string, MJCFMesh>& meshes,
                std::map<std::string, MJCFMaterial>& materials,
                std::map<std::string, MJCFTexture>& textures,
                std::string className,
                std::map<std::string, MJCFClass>& classes,
                ImportConfig& config)
{
    tinyxml2::XMLElement* m = a->FirstChildElement("mesh");
    while (m)
    {
        MJCFMesh mMesh = MJCFMesh();
        LoadMesh(m, mMesh, className, compiler, classes);

        std::string meshName = mMesh.name;
        std::string meshFile = mMesh.filename;
        Vec3 meshScale = mMesh.scale;

        // if (config.meshRootDirectory != "")
        //     baseDirPath = config.meshRootDirectory;
        std::string meshPath = baseDirPath + compiler.meshDir + "/" + meshFile;
        if (meshName == "")
        {
            if (meshFile != "")
            {
                size_t lastindex = meshFile.find_last_of(".");
                meshName = meshFile.substr(0, lastindex);
            }
            else
            {
                CARB_LOG_ERROR("*** Mesh missing name and file attributes!\n");
            }
        }

        meshes[meshName] = mMesh;

        std::map<std::string, MeshInfo>::iterator it = simulationMeshCache.find(meshName);
        Mesh* mesh = nullptr;

        if (it == simulationMeshCache.end())
        {
            Vec3 scale{ 1.f };
            mesh::MeshImporter meshImporter;
            mesh = meshImporter.loadMeshAssimp(meshPath.c_str(), scale, GymMeshNormalMode::eComputePerFace);

            if (!mesh)
            {
                CARB_LOG_ERROR("*** Failed to load '%s'!\n", meshPath.c_str());
            }

            if (meshScale.x != 1.0f || meshScale.y != 1.0f || meshScale.z != 1.0f)
            {
                mesh->Transform(ScaleMatrix(meshScale));
            }

            mesh->name = meshName;

            // use flat normals on collision shapes
            mesh->CalculateFaceNormals();
            GymMeshHandle gymMeshHandle = -1;

            MeshInfo meshInfo;
            meshInfo.mesh = mesh;
            meshInfo.meshHandle = gymMeshHandle;
            simulationMeshCache[meshName] = meshInfo;
        }
        else
        {
            mesh = it->second.mesh;
        }

        m = m->NextSiblingElement("mesh");
    }

    tinyxml2::XMLElement* mat = a->FirstChildElement("material");
    while (mat)
    {
        std::string matName = "", texture = "";
        float matSpecular = 0.5f, matShininess = 0.0f;
        Vec4 rgba = Vec4(0.2f, 0.2f, 0.2f, 1.0f);

        getIfExist(mat, "name", matName);
        getIfExist(mat, "specular", matSpecular);
        getIfExist(mat, "shininess", matShininess);
        getIfExist(mat, "texture", texture);
        getIfExist(mat, "rgba", rgba);

        MJCFMaterial material;
        material.name = matName;
        material.texture = texture;
        material.specular = matSpecular;
        material.shininess = matShininess;
        material.rgba = rgba;

        materials[matName] = material;
        mat = mat->NextSiblingElement("material");
    }

    tinyxml2::XMLElement* tex = a->FirstChildElement("texture");
    while (tex)
    {
        std::string texName = "", texFile = "", gridsize = "", gridlayout = "", type = "";

        getIfExist(tex, "name", texName);
        getIfExist(tex, "file", texFile);
        getIfExist(tex, "gridsize", gridsize);
        getIfExist(tex, "gridlayout", gridlayout);
        getIfExist(tex, "type", type);

        if (texFile != "")
        {
            texFile = baseDirPath + compiler.textureDir + "/" + texFile;
        }

        MJCFTexture texture = MJCFTexture();
        texture.name = texName;
        texture.filename = texFile;
        texture.gridsize = gridsize;
        texture.gridlayout = gridlayout;
        texture.type = type;

        textures[texName] = texture;
        tex = tex->NextSiblingElement("texture");
    }
}

void LoadGlobals(tinyxml2::XMLElement* root,
                 std::string& defaultClassName,
                 std::string baseDirPath,
                 MJCFBody& worldBody,
                 std::vector<MJCFBody*>& bodies,
                 std::vector<MJCFActuator*>& actuators,
                 std::vector<MJCFTendon*>& tendons,
                 std::vector<MJCFContact*>& contacts,
                 std::map<std::string, MeshInfo>& simulationMeshCache,
                 std::map<std::string, MJCFMesh>& meshes,
                 std::map<std::string, MJCFMaterial>& materials,
                 std::map<std::string, MJCFTexture>& textures,
                 MJCFCompiler& compiler,
                 std::map<std::string, MJCFClass>& classes,
                 std::map<std::string, int>& jointToActuatorIdx,
                 ImportConfig& config)
{
    // parses attributes for the MJCF compiler, which defines settings such as
    // angle units (rad/deg), mesh directory path, etc.
    LoadCompiler(root->FirstChildElement("compiler"), compiler);

    // reset counters
    bodyIdxCount = 0;
    geomIdxCount = 0;
    siteIdxCount = 0;
    jointIdxCount = 0;

    // deal with defaults
    tinyxml2::XMLElement* d = root->FirstChildElement("default");
    if (!d)
    {
        // if no default, set the defaultClassName to default if it does not exist
        // yet. added this condition to avoid overwriting default class parameters
        // parsed in a prior call
        if (classes.find(defaultClassName) == classes.end())
        {
            classes[defaultClassName] = MJCFClass();
        }
    }
    else
    {
        // only handle one top level default
        if (d->Attribute("class"))
            defaultClassName = d->Attribute("class");
        classes[defaultClassName] = MJCFClass();
        LoadDefault(d, defaultClassName, classes[defaultClassName], compiler, classes);
        if (d->NextSiblingElement("default"))
        {
            CARB_LOG_ERROR("*** Can only handle one top level default at the moment!");
            return;
        }
    }

    tinyxml2::XMLElement* a = root->FirstChildElement("asset");
    if (a)
    {
        {
            tinyxml2::XMLDocument includeDoc;
            tinyxml2::XMLElement* includeRoot = LoadInclude(includeDoc, a->FirstChildElement("include"), baseDirPath);
            if (includeRoot)
            {
                LoadAssets(includeRoot, baseDirPath, compiler, simulationMeshCache, meshes, materials, textures,
                           defaultClassName, classes, config);
            }
        }

        LoadAssets(a, baseDirPath, compiler, simulationMeshCache, meshes, materials, textures, defaultClassName,
                   classes, config);
    }

    // finds the origin of the world frame within which the rest of the kinematic
    // tree is defined
    tinyxml2::XMLElement* wb = root->FirstChildElement("worldbody");
    if (wb)
    {
        {
            tinyxml2::XMLDocument includeDoc;
            tinyxml2::XMLElement* includeRoot = LoadInclude(includeDoc, wb->FirstChildElement("include"), baseDirPath);
            if (includeRoot)
            {
                tinyxml2::XMLElement* c = includeRoot->FirstChildElement("body");
                while (c)
                {
                    bodies.push_back(new MJCFBody());
                    LoadBody(c, bodies, *bodies.back(), defaultClassName, compiler, classes, baseDirPath);
                    c = c->NextSiblingElement("body");
                }
            }
        }

        tinyxml2::XMLElement* c = wb->FirstChildElement("body");
        while (c)
        {
            bodies.push_back(new MJCFBody());
            LoadBody(c, bodies, *bodies.back(), defaultClassName, compiler, classes, baseDirPath);
            c = c->NextSiblingElement("body");
        }

        worldBody = MJCFBody();
        // load sites and geoms
        tinyxml2::XMLElement* g = wb->FirstChildElement("geom");
        while (g)
        {
            worldBody.geoms.push_back(new MJCFGeom());
            LoadGeom(g, *worldBody.geoms.back(), defaultClassName, compiler, classes, true);
            if (worldBody.geoms.back()->type == MJCFGeom::OTHER)
            {
                // don't know how to deal with it - remove it from list
                worldBody.geoms.pop_back();
            }
            g = g->NextSiblingElement("geom");
        }
        tinyxml2::XMLElement* s = wb->FirstChildElement("site");
        while (s)
        {
            worldBody.sites.push_back(new MJCFSite());
            LoadSite(wb->FirstChildElement("site"), *worldBody.sites.back(), defaultClassName, compiler, classes, true);
            s = s->NextSiblingElement("site");
        }
    }

    tinyxml2::XMLElement* ac = root->FirstChildElement("actuator");
    if (ac)
    {
        tinyxml2::XMLElement* c = ac->FirstChildElement();
        while (c)
        {
            MJCFActuator::Type type;
            std::string elementName{ c->Name() };
            if (elementName == "motor")
            {
                type = MJCFActuator::MOTOR;
            }
            else if (elementName == "position")
            {
                type = MJCFActuator::POSITION;
            }
            else if (elementName == "velocity")
            {
                type = MJCFActuator::VELOCITY;
            }
            else if (elementName == "general")
            {
                type = MJCFActuator::GENERAL;
            }
            else
            {
                CARB_LOG_ERROR("*** Only motor, position, velocity actuators supported");
                c = c->NextSiblingElement();
                continue;
            }

            MJCFActuator* actuator = new MJCFActuator();
            LoadActuator(c, *actuator, defaultClassName, type, classes);
            jointToActuatorIdx[actuator->joint] = int(actuators.size());
            actuators.push_back(actuator);
            c = c->NextSiblingElement();
        }
    }

    // load tendons
    tinyxml2::XMLElement* tc = root->FirstChildElement("tendon");
    if (tc)
    {
        {
            // parse fixed tendons first
            tinyxml2::XMLElement* c = tc->FirstChildElement("fixed");
            while (c)
            {
                MJCFTendon* tendon = new MJCFTendon();
                LoadTendon(c, *tendon, defaultClassName, MJCFTendon::FIXED, classes);
                tendons.push_back(tendon);
                c = c->NextSiblingElement("fixed");
            }
        }
        {
            // parse spatial tendons next
            tinyxml2::XMLElement* c = tc->FirstChildElement("spatial");
            while (c)
            {
                MJCFTendon* tendon = new MJCFTendon();
                LoadTendon(c, *tendon, defaultClassName, MJCFTendon::SPATIAL, classes);
                tendons.push_back(tendon);
                c = c->NextSiblingElement("spatial");
            }
        }
    }

    tinyxml2::XMLElement* cc = root->FirstChildElement("contact");
    if (cc)
    {
        tinyxml2::XMLElement* c = cc->FirstChildElement();
        while (c)
        {
            MJCFContact::Type type;
            std::string elementName{ c->Name() };
            if (elementName == "pair")
            {
                type = MJCFContact::PAIR;
            }
            else if (elementName == "exclude")
            {
                type = MJCFContact::EXCLUDE;
            }
            else
            {
                CARB_LOG_ERROR("*** Invalid contact specification");
                c = c->NextSiblingElement();
                continue;
            }

            MJCFContact* contact = new MJCFContact();
            LoadContact(c, *contact, type, classes);
            contacts.push_back(contact);
            c = c->NextSiblingElement();
        }
    }
}

} // namespace mjcf
} // namespace importer
} // namespace omni
