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
#include <rviz/frame_manager.h>

#include "marker_detection/marker_detection_visual.h"
#include "marker_detection/marker_detection_display.h"

namespace marker_rviz_plugin {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
MarkerDetectionDisplay::MarkerDetectionDisplay() {

    std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);

}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void MarkerDetectionDisplay::onInitialize() {
    MFDClass::onInitialize();

    // create ManualObject
    //Ogre::ManualObject* manual = context_->getSceneManager()->createManualObject("manual");


        /*
    // specify the material (by name) and rendering type
    manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );

    // define start and end point
    manual->position(-0.2, -0.2, -0.2);
    manual->position(0.2, 0.2, 0.2);

    // tell Ogre, your definition has finished
    manual->end();
        */

        /*
        Ogre::ResourceManager::ResourceMapIterator materialIterator = Ogre::MaterialManager::getSingleton().getResourceIterator();
        while (materialIterator.hasMoreElements())
        {

            std::string str = (static_cast<Ogre::ResourcePtr>(materialIterator.peekNextValue()))->getName();
            std::cout << "c: " << str << std::endl;
            materialIterator.moveNext();
        }
        */
        Ogre::ResourceGroupManager::ResourceDeclarationList::iterator it;
        Ogre::ResourceGroupManager::ResourceDeclarationList resources =   Ogre::ResourceGroupManager::getSingleton().getResourceDeclarationList(
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME
        );

        for(it = resources.begin();it != resources.end(); ++it) {
            std::cout << "************************"   << it->resourceName;
        }

    Ogre::Plane plane;
    plane.normal = Ogre::Vector3::UNIT_Y;
    plane.d = 0;

    Ogre::MeshManager::getSingleton().createPlane("image",
    Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    plane,
    0.3, 0.3,
    1, 1, true, 1,
    1.0, 1.0,
    Ogre::Vector3::UNIT_Z);

    Ogre::Entity* planeEntity = context_->getSceneManager()->createEntity("image");
    planeEntity->setMaterialName("RVIZ/Red");
    planeEntity->setCastShadows(false);



    Ogre::MaterialPtr bg_material_ = Ogre::MaterialManager::getSingleton().create( "imageMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    bg_material_->setDepthWriteEnabled(false);
    bg_material_->setReceiveShadows(false);
    bg_material_->setDepthCheckEnabled(false);
    bg_material_->getTechnique(0)->setLightingEnabled(false);


    Ogre::TextureUnitState* tu = bg_material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName("textures/marker_icon.png");
    tu->setTextureFiltering( Ogre::TFO_NONE );
    tu->setAlphaOperation( Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL, Ogre::LBS_CURRENT, 0.0 );

    //context_->getSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(planeEntity);

    planeEntity->setMaterial(bg_material_);


    // add ManualObject to the RootSceneNode (so it will be visible)
    scene_node_->attachObject(planeEntity);
}

MarkerDetectionDisplay::~MarkerDetectionDisplay() {
}

// Clear the visuals by deleting their objects.
void MarkerDetectionDisplay::reset() {
    MFDClass::reset();
}

// This is our callback to handle an incoming message.
void MarkerDetectionDisplay::processMessage ( const marker_msgs::MarkerDetection::ConstPtr& msg ) {

    std::cout << "msg" << std::endl;
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

    scene_node_->setPosition( position );
    scene_node_->setOrientation( orientation );


        /*
    for ( size_t i = 0; i < msg->markers.size(); i++ ) {
        double p_x = msg->markers[i].pose.position.x;
        double p_y = msg->markers[i].pose.position.y;
        double p_z = msg->markers[i].pose.position.z;
        double o_x = msg->markers[i].pose.orientation.x;
        double o_y = msg->markers[i].pose.orientation.y;
        double o_z = msg->markers[i].pose.orientation.z;
        double o_w = msg->markers[i].pose.orientation.w;

        markers_[i].reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        markers_[i]->setColor ( color_ );
        markers_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, p_z ) );
        markers_[i]->setOrientation ( Ogre::Quaternion ( o_w, o_x, o_y, o_z ) );
        markers_[i]->setScale ( Ogre::Vector3 ( scale_, scale_, scale_ ) );
    }
        */


    context_->queueRender();
}

} // end namespace marker_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( marker_rviz_plugin::MarkerDetectionDisplay,rviz::Display )
