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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include "rviz/ogre_helpers/axes.h"

#include "marker_with_covariance/marker_with_covariance_visual.h"

namespace marker_rviz_plugin {

    MarkerWithCovarianceVisual::MarkerWithCovarianceVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node) {
        scene_manager_ = scene_manager;
        frame_node_ = parent_node->createChildSceneNode();

        _showAxes = true;
        _showMarker = true;
        _showLabel = true;
        _scale = 1;
    }

    MarkerWithCovarianceVisual::~MarkerWithCovarianceVisual() {
        // Destroy the frame node since we don't need it anymore.
        scene_manager_->destroySceneNode(frame_node_);
    }

    void MarkerWithCovarianceVisual::setMessage(const marker_msgs::MarkerWithCovarianceStamped::ConstPtr &msg) {
        marker_msgs::MarkerWithCovariance marker_cov = msg->marker;
        marker_msgs::Marker marker = marker_cov.marker;

        double p_x = marker.pose.position.x;
        double p_y = marker.pose.position.y;
        double p_z = marker.pose.position.z;
        double o_x = marker.pose.orientation.x;
        double o_y = marker.pose.orientation.y;
        double o_z = marker.pose.orientation.z;
        double o_w = marker.pose.orientation.w;

        int id = -1;
        if (marker.ids.size() > 0)
            id = marker.ids[0];

        MarkerWithCovariance *m = new MarkerWithCovariance(scene_manager_, frame_node_, id);
        m->setPosition(Ogre::Vector3(p_x, p_y, p_z));
        m->setOrientation(Ogre::Quaternion(o_w, o_x, o_y, o_z));
        m->setShowMarker(_showMarker);
        m->setShowAxes(_showAxes);
        m->setShowLabel(_showLabel);
        m->setScale(Ogre::Vector3(_scale, _scale, _scale));
        m->setCovarianceMatrix(msg->marker.covariance);
        _marker.reset(m);
    }

    void MarkerWithCovarianceVisual::setFramePosition(const Ogre::Vector3 &position) {
        frame_node_->setPosition(position);
    }

    void MarkerWithCovarianceVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
        frame_node_->setOrientation(orientation);
    }

    void MarkerWithCovarianceVisual::setShowAxes(bool showAxes) {
        _marker->setShowAxes(showAxes);
        _showAxes = showAxes;
    }

    void MarkerWithCovarianceVisual::setShowMarker(bool showMarker) {
        _marker->setShowMarker(showMarker);
        _showMarker = showMarker;
    }

    void MarkerWithCovarianceVisual::setShowLabel(bool showLabel) {
        _marker->setShowLabel(showLabel);
        _showLabel = showLabel;
    }

    void MarkerWithCovarianceVisual::setScale(float scale) {
        _marker->setScale(Ogre::Vector3(scale, scale, scale));
        _scale = scale;
    }

}