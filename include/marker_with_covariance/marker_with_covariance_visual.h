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

#ifndef MARKER_WITH_COVARIANCE_VISUAL_H
#define MARKER_WITH_COVARIANCE_VISUAL_H

#include <marker_msgs/MarkerWithCovarianceStamped.h>
#include "ogre_visuals/marker.h"
#include "ogre_visuals/marker_with_covariance.h"

namespace Ogre {
    class Vector3;

    class Quaternion;
}

namespace rviz {
    class Axes;
}

namespace marker_rviz_plugin {

    class MarkerWithCovarianceVisual {
    public:
        MarkerWithCovarianceVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

        virtual ~MarkerWithCovarianceVisual();

        // Configure the visual to show the data in the message.
        void setMessage(const marker_msgs::MarkerWithCovarianceStamped::ConstPtr &msg);

        // Set the pose of the coordinate frame the message refers to.
        // These could be done inside setMessage(), but that would require
        // calls to FrameManager and error handling inside setMessage(),
        // which doesn't seem as clean.  This way MarkerDetectionVisual is
        // only responsible for visualization.
        void setFramePosition(const Ogre::Vector3 &position);

        void setFrameOrientation(const Ogre::Quaternion &orientation);

        void setShowAxes(bool showAxes);

        void setShowMarker(bool showMarker);

        void setShowLabel(bool showLabel);

        void setScale(float scale);

    private:
        boost::shared_ptr<Marker> _marker;

        Ogre::SceneNode *frame_node_;
        Ogre::SceneManager *scene_manager_;

        bool _showAxes;
        bool _showMarker;
        bool _showLabel;
        float _scale;
    };

} // end namespace marker_rviz_plugin

#endif // MARKER_WITH_COVARIANCE_VISUAL_H
