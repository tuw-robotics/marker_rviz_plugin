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

#include "marker_detection/marker_detection_visual.h"

namespace marker_rviz_plugin {

    MarkerDetectionVisual::MarkerDetectionVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node) {
        scene_manager_ = scene_manager;
        frame_node_ = parent_node->createChildSceneNode();

        _showAxes = true;
        _showMarker = true;
        _showLabel = true;
    	_colorLabel  = Ogre::ColourValue ( 0, 170, 0 );
        _scale = 1;
    }

    MarkerDetectionVisual::~MarkerDetectionVisual() {
        // Destroy the frame node since we don't need it anymore.
        scene_manager_->destroySceneNode(frame_node_);
    }

    void MarkerDetectionVisual::setMessage(const marker_msgs::MarkerDetection::ConstPtr &msg) {
        _markers.resize(msg->markers.size());

        for (size_t i = 0; i < msg->markers.size(); i++) {
            double p_x = msg->markers[i].pose.position.x;
            double p_y = msg->markers[i].pose.position.y;
            double p_z = msg->markers[i].pose.position.z;
            double o_x = msg->markers[i].pose.orientation.x;
            double o_y = msg->markers[i].pose.orientation.y;
            double o_z = msg->markers[i].pose.orientation.z;
            double o_w = msg->markers[i].pose.orientation.w;

            int id = -1;
            if (msg->markers[i].ids.size() > 0)
                id = msg->markers[i].ids[0];

            _markers[i].reset(new Marker(scene_manager_, frame_node_, id));
            _markers[i]->setPosition(Ogre::Vector3(p_x, p_y, p_z));
            _markers[i]->setOrientation(Ogre::Quaternion(o_w, o_x, o_y, o_z));
            _markers[i]->setShowMarker(_showMarker);
            _markers[i]->setShowAxes(_showAxes);
            _markers[i]->setShowLabel(_showLabel);
            _markers[i]->setColorLabel(_colorLabel);
            _markers[i]->setScale(Ogre::Vector3(_scale, _scale, _scale));
        }
    }

    void MarkerDetectionVisual::setFramePosition(const Ogre::Vector3 &position) {
        frame_node_->setPosition(position);
    }

    void MarkerDetectionVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
        frame_node_->setOrientation(orientation);
    }

    void MarkerDetectionVisual::setShowAxes(bool showAxes) {
        for (size_t i = 0; i < _markers.size(); i++) {
            _markers[i]->setShowAxes(showAxes);
        }

        _showAxes = showAxes;
    }

    void MarkerDetectionVisual::setShowMarker(bool showMarker) {
        for (size_t i = 0; i < _markers.size(); i++) {
            _markers[i]->setShowMarker(showMarker);
        }

        _showMarker = showMarker;
    }

    void MarkerDetectionVisual::setShowLabel(bool showLabel) {
        for (size_t i = 0; i < _markers.size(); i++) {
            _markers[i]->setShowLabel(showLabel);
        }

        _showLabel = showLabel;
    }
    void MarkerDetectionVisual::setColorLabel(Ogre::ColourValue color) {
	_colorLabel = color;
        for (size_t i = 0; i < _markers.size(); i++) {
            _markers[i]->setColorLabel(_colorLabel);
        }
    }

    void MarkerDetectionVisual::setScale(float scale) {
        for (size_t i = 0; i < _markers.size(); i++) {
            _markers[i]->setScale(Ogre::Vector3(scale, scale, scale));
        }

        _scale = scale;
    }

}
