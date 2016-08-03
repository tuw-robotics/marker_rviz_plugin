/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include "rviz/ogre_helpers/axes.h"

#include "marker_detection/marker_detection_visual.h"

namespace marker_rviz_plugin {

MarkerDetectionVisual::MarkerDetectionVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();

    _showAxes = true;
    _showMarker = true;
}

MarkerDetectionVisual::~MarkerDetectionVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void MarkerDetectionVisual::setMessage ( const marker_msgs::MarkerDetection::ConstPtr& msg ) {
    _markersAxes.resize(msg->markers.size());
    _markersImages.resize(msg->markers.size());

    for ( size_t i = 0; i < msg->markers.size(); i++ ) {
        double p_x = msg->markers[i].pose.position.x;
        double p_y = msg->markers[i].pose.position.y;
        double p_z = msg->markers[i].pose.position.z;
        double o_x = msg->markers[i].pose.orientation.x;
        double o_y = msg->markers[i].pose.orientation.y;
        double o_z = msg->markers[i].pose.orientation.z;
        double o_w = msg->markers[i].pose.orientation.w;

        // Create imagePlane scene node
        // FIXME: Segfault occurs here if setMessage is called twice
        // I guess this is because reset deletes the reference to the last SceneNode and the attached Entity,
        // but the attached Entity is never cleaned up properly. This will change when this part gets changed into
        // an own proper class.
        Ogre::Entity* markerEntity = scene_manager_->createEntity("imagePlane");
        markerEntity->setCastShadows(false);
        markerEntity->setMaterialName("imagePlaneMaterial");
        Ogre::SceneNode *markerNode = frame_node_->createChildSceneNode();
        markerNode->attachObject(markerEntity);

        _markersImages[i].reset(markerNode);
        _markersImages[i]->setPosition(Ogre::Vector3(p_x, p_y, p_z));
        _markersImages[i]->setOrientation(Ogre::Quaternion(o_w, o_x, o_y, o_z));
        _markersImages[i]->setVisible(_showMarker);

        _markersAxes[i].reset(new rviz::Axes(scene_manager_, frame_node_, 0.2, 0.02));
        _markersAxes[i]->setPosition(Ogre::Vector3(p_x, p_y, p_z));
        _markersAxes[i]->setOrientation(Ogre::Quaternion(o_w, o_x, o_y, o_z));
        _markersAxes[i]->getSceneNode()->setVisible(_showAxes);
    }

}

// Position is passed through to the SceneNode.
void MarkerDetectionVisual::setFramePosition(const Ogre::Vector3 &position) {
    frame_node_->setPosition(position);
}

// Orientation is passed through to the SceneNode.
void MarkerDetectionVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
    frame_node_->setOrientation(orientation);
}

void MarkerDetectionVisual::setShowAxes(bool showAxes) {
    for (size_t i = 0; i < _markersAxes.size(); i++) {
        _markersAxes[i]->getSceneNode()->setVisible(showAxes);
    }

    _showAxes = showAxes;
}

void MarkerDetectionVisual::setShowMarker(bool showMarker) {
    for (size_t i = 0; i < _markersImages.size(); i++) {
        _markersImages[i]->setVisible(showMarker);
    }

    _showMarker = showMarker;
}

}
