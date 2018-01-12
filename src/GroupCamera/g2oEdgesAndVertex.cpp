#include "g2oEdgesAndVertex.h"

namespace ORB_SLAM2
{

    Eigen::Vector2d project2d(const Eigen::Vector3d& v)  {
        Eigen::Vector2d res;
        res(0) = v(0)/v(2);
        res(1) = v(1)/v(2);
        return res;
    }

    Eigen::Vector3d unproject2d(const Eigen::Vector2d& v)  {
        Eigen::Vector3d res;
        res(0) = v(0);
        res(1) = v(1);
        res(2) = 1;
        return res;
    }



    bool EdgeSE3ProjectXYZOnlyPoseGroupCamera::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZOnlyPoseGroupCamera::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZOnlyPoseGroupCamera::linearizeOplus() {
        g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;


        Eigen::Matrix<double,2,3> dp_dXc;
        dp_dXc<<
             fx*invz,0,-fx*x*invz_2,
                0,fy*invz,-fy*y*invz_2;
        Eigen::Matrix<double,3,6> dXc_dXi;
        dXc_dXi<<
               0,-z,y,-1,0,0,
                z,0,-x,0,-1,0,
                -y,x,0,0,0,-1;

        _jacobianOplusXi = dp_dXc*Tcg.rotation().toRotationMatrix()*dXc_dXi;
    }



    Eigen::Vector2d EdgeSE3ProjectXYZOnlyPoseGroupCamera::cam_project(const Eigen::Vector3d & trans_xyz) const{
        Eigen::Vector2d proj = project2d(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }












    EdgeSE3ProjectXYZGroupCamera::EdgeSE3ProjectXYZGroupCamera() : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeSE3ProjectXYZGroupCamera::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZGroupCamera::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZGroupCamera::linearizeOplus() {
        g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ* vi = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);


        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double invz = 1.0/xyz_trans[2];
        double invz_2 = invz*invz;

        Eigen::Matrix<double,2,3> dp_dXc;
        dp_dXc<<
              fx*invz,0,-fx*x*invz_2,
                0,fy*invz,-fy*y*invz_2;


        _jacobianOplusXi =  dp_dXc*T.rotation().toRotationMatrix() * T.rotation().toRotationMatrix();
        Eigen::Matrix<double,3,6> dXc_dXi;
        dXc_dXi<<
               0,-z,y,-1,0,0,
                z,0,-x,0,-1,0,
                -y,x,0,0,0,-1;

        _jacobianOplusXj = dp_dXc*Tcg.rotation().toRotationMatrix()*dXc_dXi;

    }

    Eigen::Vector2d EdgeSE3ProjectXYZGroupCamera::cam_project(const Eigen::Vector3d & trans_xyz) const{
        Eigen::Vector2d proj = project2d(trans_xyz);
        Eigen::Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }








}