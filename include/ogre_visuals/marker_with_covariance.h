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

#ifndef MARKER_RVIZ_PLUGIN_MARKER_WITH_COVARIANCE_H
#define MARKER_RVIZ_PLUGIN_MARKER_WITH_COVARIANCE_H

#include <boost/array.hpp>
#include <Ogre.h>
#include "ogre_visuals/marker.h"

namespace marker_rviz_plugin {

    class MarkerWithCovariance : public Marker {
    public:

        MarkerWithCovariance(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node = 0, int id = -1);

        virtual ~MarkerWithCovariance();

        virtual void setCovarianceMatrix(boost::array<double, 36> m);

        virtual void setScale(const Ogre::Vector3 &scale);

    protected:
        Ogre::SceneNode *variance_pos_parent;
        rviz::Shape *variance_pos_;
        rviz::Shape *variance_rpy_[3];

    };

}

#endif //MARKER_RVIZ_PLUGIN_MARKER_WITH_COVARIANCE_H
