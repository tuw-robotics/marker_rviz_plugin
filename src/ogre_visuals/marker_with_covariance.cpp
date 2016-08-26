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

#include "ogre_visuals/marker_with_covariance.h"
#include "rviz/ogre_helpers/shape.h"

namespace marker_rviz_plugin {

    MarkerWithCovariance::MarkerWithCovariance(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node, int id)
            : Marker(scene_manager, parent_node, id) {

        variance_pos_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, scene_node_);
        variance_pos_->setColor(Ogre::ColourValue(1.0, 1.0, 0.0));
        variance_pos_->getMaterial()->setReceiveShadows(false);
    }

    MarkerWithCovariance::~MarkerWithCovariance() {
        delete variance_pos_;
    }

    void MarkerWithCovariance::setCovarianceMatrix(boost::array<double, 36> m) {
        Ogre::Matrix3 cov_xyz = Ogre::Matrix3(
            m[6 * 0 + 0], m[6 * 0 + 1], m[6 * 0 + 2],
            m[6 * 1 + 0], m[6 * 1 + 1], m[6 * 1 + 2],
            m[6 * 2 + 0], m[6 * 2 + 1], m[6 * 2 + 2]
        );

        Ogre::Real eigenvalues[3];
        Ogre::Vector3 eigenvectors[3];
        cov_xyz.EigenSolveSymmetric(eigenvalues, eigenvectors);

        if (eigenvalues[0] < 0)
            eigenvalues[0] = 0;

        if (eigenvalues[1] < 0)
            eigenvalues[1] = 0;

        if (eigenvalues[2] < 0)
            eigenvalues[2] = 0;


        variance_pos_->setOrientation(Ogre::Quaternion(eigenvectors[0], eigenvectors[1], eigenvectors[2]));
        variance_pos_->setScale(
            Ogre::Vector3(
                fmax(2 * sqrt(eigenvalues[0]), 0.005f), // Try to avoid a complete flat sphere
                fmax(2 * sqrt(eigenvalues[1]), 0.005f),
                fmax(2 * sqrt(eigenvalues[2]), 0.005f)
            )
        );
    }

}
