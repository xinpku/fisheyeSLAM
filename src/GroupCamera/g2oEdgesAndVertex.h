#pragma once


#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

    class  EdgeSE3ProjectXYZGroupCamera: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZGroupCamera();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError()  {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs-cam_project((Tcg_*v1->estimate()).map(v2->estimate()));
            //_error = obs-cam_project((Tcg_*v1->estimate()).map(v2->estimate()));

        }

        bool isDepthPositive() {
            const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
            const g2o::VertexSBAPointXYZ* v2 = static_cast<const g2o::VertexSBAPointXYZ*>(_vertices[0]);
            return ((Tcg_*v1->estimate()).map(v2->estimate()))(2)>0.0;
        }


        virtual void linearizeOplus();

        Eigen::Vector2d cam_project(const Eigen::Vector3d & trans_xyz) const;

        double fx, fy, cx, cy;
        g2o::SE3Quat Tcg_;
    };




    class EdgeSE3ProjectXYZOnlyPoseGroupCamera : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZOnlyPoseGroupCamera()
        {}

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        void computeError()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            Eigen::Vector2d obs(_measurement);
            _error = obs - cam_project((Tcg_*v1->estimate()).map(Xw));
        }

        bool isDepthPositive()
        {
            const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
            return ((Tcg_*v1->estimate()).map(Xw))(2) > 0.0;
        }


        virtual void linearizeOplus();

        Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz) const;

        Eigen::Vector3d Xw;
        double fx, fy, cx, cy;
        g2o::SE3Quat Tcg_;
    };


}



