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

#ifndef MARKER_WITH_COVARIANCE_ARRAY_DISPLAY_H
#define MARKER_WITH_COVARIANCE_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN

#include <marker_msgs/MarkerWithCovarianceArray.h>
#include <rviz/message_filter_display.h>
#include <boost/circular_buffer.hpp>

#endif

#include <rviz/properties/enum_property.h>

namespace Ogre {
    class SceneNode;
}

namespace rviz {
    class ColorProperty;

    class EnumProperty;

    class FloatProperty;

    class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace marker_rviz_plugin {

    class MarkerWithCovarianceArrayVisual;

// Here we declare our new subclass of rviz::Display.  Every display
// which can be listed in the "Displays" panel is a subclass of
// rviz::Display.
    class MarkerWithCovarianceArrayDisplay : public rviz::MessageFilterDisplay<marker_msgs::MarkerWithCovarianceArray> {
    Q_OBJECT
    public:
        // Constructor.  pluginlib::ClassLoader creates instances by calling
        // the default constructor, so make sure you have one.
        MarkerWithCovarianceArrayDisplay();

        virtual ~MarkerWithCovarianceArrayDisplay();

        // Overrides of protected virtual functions from Display.  As much
        // as possible, when Displays are not enabled, they should not be
        // subscribed to incoming data and should not show anything in the
        // 3D view.  These functions are where these connections are made
        // and broken.
    protected:
        virtual void onInitialize();

        // A helper to clear this display back to the initial state.
        virtual void reset();

        // These Qt slots get connected to signals indicating changes in the user-editable properties.
    private Q_SLOTS:

        void updateVisual();


    private:
        // Function to handle an incoming ROS message.
        void processMessage(const marker_msgs::MarkerWithCovarianceArray::ConstPtr &msg);

        rviz::BoolProperty *_showAxesProperty;
        rviz::BoolProperty *_showMarkerProperty;
        rviz::BoolProperty *_showLabelProperty;
        rviz::FloatProperty *_markerSizeProperty;

        MarkerWithCovarianceArrayVisual *_visual;
    };

} // end namespace marker_rviz_plugin

#endif // MARKER_WITH_COVARIANCE_ARRAY_DISPLAY_H
// %EndTag(FULL_SOURCE)%
