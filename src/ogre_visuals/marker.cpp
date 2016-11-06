/*
 * Copyright (c) 2016, Lukas Pfeifhofer <lukas.pfeifhofer@devlabs.pro>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ogre_visuals/marker.h"

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMeshManager.h>

namespace marker_rviz_plugin {

    MarkerResources Marker::static_resources_;

    MarkerResources::MarkerResources() {
        // Create imagePlane mesh
        Ogre::Plane imagePlane;
        imagePlane.normal = Ogre::Vector3::UNIT_Z;
        imagePlane.d = 0;

        Ogre::MeshManager::getSingleton().createPlane("imagePlane",
                                                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                      imagePlane,
                                                      1.0, 1.0,
                                                      1, 1, true, 1,
                                                      1.0, 1.0,
                                                      Ogre::Vector3::UNIT_X);

        // Create image material and load texture
        Ogre::MaterialPtr planeMaterial = Ogre::MaterialManager::getSingleton().create("imagePlaneMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        planeMaterial->setCullingMode(Ogre::CULL_NONE);
        planeMaterial->setSceneBlending(Ogre::SBT_REPLACE);
        planeMaterial->setReceiveShadows(false);
        planeMaterial->getTechnique(0)->setLightingEnabled(false);

        Ogre::TextureUnitState *tu = planeMaterial->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName("textures/marker_rect_icon.png");
        tu->setTextureFiltering(Ogre::TFO_NONE);
        tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);
    }

    MarkerResources::~MarkerResources() {

    }

    Marker::Marker(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node, int id)
            : rviz::Object(scene_manager) {
        if (!parent_node) {
            parent_node = scene_manager_->getRootSceneNode();
        }

        scene_node_ = parent_node->createChildSceneNode();

        markerNode_ = scene_node_->createChildSceneNode();
        markerEntity_ = scene_manager_->createEntity("imagePlane");
        markerEntity_->setCastShadows(false);
        markerEntity_->setMaterialName("imagePlaneMaterial");
        markerNode_->attachObject(markerEntity_);
        markerNode_->setScale(1.0, 1.0, 1.0);

        axes_ = new rviz::Axes(scene_manager_, scene_node_, 0.7, 0.07);

        std::stringstream ss;
        if (id >= 0) {
            ss << "#" << id;
        } else {
            ss << "-";
        }
        text_ = new rviz::MovableText(ss.str(), "Arial", 0.4);
        text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);
        text_->setColor(Ogre::ColourValue(0.70, 0.70, 0.70));

        text_node_ = scene_node_->createChildSceneNode();
        text_node_->setPosition(0.20, 0.20, 0.30);
        text_node_->attachObject(text_);
    }

    Marker::~Marker() {
        delete axes_;
        delete text_;

        if(markerNode_)
            scene_manager_->destroySceneNode(markerNode_);

        if (markerEntity_)
            scene_manager_->destroyEntity(markerEntity_);

        scene_manager_->destroySceneNode(scene_node_->getName());
    }

    void Marker::setShowAxes(bool showAxes) {
        axes_->getSceneNode()->setVisible(showAxes);
    }

    void Marker::setShowMarker(bool showMarker) {
        markerEntity_->setVisible(showMarker);
    }

    void Marker::setShowLabel(bool showLabel) {
        text_node_->setVisible(showLabel);
    }
    void Marker::setColorLabel(Ogre::ColourValue color) {
        text_->setColor(color);
    }

    void Marker::setColor(float r, float g, float b, float a) {}

    void Marker::setPosition(const Ogre::Vector3 &position) {
        scene_node_->setPosition(position);
    }

    void Marker::setOrientation(const Ogre::Quaternion &orientation) {
        scene_node_->setOrientation(orientation);
    }

    void Marker::setScale(const Ogre::Vector3 &scale) {
        scene_node_->setScale(scale);
    }

    const Ogre::Vector3 &Marker::getPosition() {
        return scene_node_->getPosition();
    }

    const Ogre::Quaternion &Marker::getOrientation() {
        return scene_node_->getOrientation();
    }

    void Marker::setUserData(const Ogre::Any &data) {
        axes_->setUserData(data);

        if (markerEntity_)
            markerEntity_->getUserObjectBindings().setUserAny(data);
    }

}
