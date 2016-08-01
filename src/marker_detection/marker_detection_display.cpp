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

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/package.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/frame_manager.h>
#include "rviz/ogre_helpers/axes.h"

#include "marker_detection/marker_detection_visual.h"
#include "marker_detection/marker_detection_display.h"

namespace marker_rviz_plugin {

MarkerDetectionDisplay::MarkerDetectionDisplay() {

    _showAxesProperty = new rviz::BoolProperty("Show Axes", true, "Show or hide axes.", this, SLOT (updateAxes()));
    _showMarkerProperty = new rviz::BoolProperty("Show Marker", true, "Show or hide marker image.", this, SLOT (updateMarker()));

    // Add the plugin orge_media folder to the Ogre ResourceGroup so it is possible to access plugin textures later on
    std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

}

void MarkerDetectionDisplay::onInitialize() {
    MFDClass::onInitialize();

    // Create imagePlane mesh
    Ogre::Plane imagePlane;
    imagePlane.normal = Ogre::Vector3::UNIT_Z;
    imagePlane.d = 0;

    Ogre::MeshManager::getSingleton().createPlane("imagePlane", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, imagePlane,
    0.3, 0.3,
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
    tu->setTextureName("textures/marker_icon.png");
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0);


    _visual = new MarkerDetectionVisual ( context_->getSceneManager(), scene_node_ );
}

MarkerDetectionDisplay::~MarkerDetectionDisplay() {
}

// Clear the visuals by deleting their objects.
void MarkerDetectionDisplay::reset() {
    MFDClass::reset();
}

void MarkerDetectionDisplay::updateAxes() {
    bool showAxes = _showAxesProperty->getBool();
    _visual->setShowAxes(showAxes);
}

void MarkerDetectionDisplay::updateMarker() {
    bool showMarker = _showMarkerProperty->getBool();
    _visual->setShowMarker(showMarker);
}

// This is our callback to handle an incoming message.
void MarkerDetectionDisplay::processMessage ( const marker_msgs::MarkerDetection::ConstPtr& msg ) {

    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id,
            msg->header.stamp,
            position, orientation ) ) {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    // Now set or update the contents of the chosen visual.
    _visual->setMessage(msg);
    _visual->setFramePosition(position);
    _visual->setFrameOrientation(orientation);
    _visual->setShowAxes(_showAxesProperty->getBool());
    _visual->setShowMarker(_showMarkerProperty->getBool());

    context_->queueRender();
}

} // end namespace marker_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( marker_rviz_plugin::MarkerDetectionDisplay,rviz::Display )
