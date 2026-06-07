/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Converter.h"

#include <cmath>
#include <iostream>

namespace ORB_SLAM3
{

namespace
{

bool HasCvMatShape(const cv::Mat &M, const int rows, const int cols, const char *name)
{
    if(M.empty() || M.rows < rows || M.cols < cols)
    {
        std::cerr << "[Converter] invalid " << name << " shape: "
                  << M.rows << "x" << M.cols << std::endl;
        return false;
    }
    return true;
}

double CvMatValueAsDouble(const cv::Mat &M, const int r, const int c)
{
    switch(M.depth())
    {
    case CV_32F:
        return static_cast<double>(M.at<float>(r, c));
    case CV_64F:
        return M.at<double>(r, c);
    case CV_32S:
        return static_cast<double>(M.at<int>(r, c));
    case CV_16S:
        return static_cast<double>(M.at<short>(r, c));
    case CV_16U:
        return static_cast<double>(M.at<unsigned short>(r, c));
    case CV_8S:
        return static_cast<double>(M.at<signed char>(r, c));
    case CV_8U:
        return static_cast<double>(M.at<unsigned char>(r, c));
    default:
        return 0.0;
    }
}

double CvVectorValueAsDouble(const cv::Mat &M, const int idx)
{
    if(M.cols == 1)
        return CvMatValueAsDouble(M, idx, 0);
    return CvMatValueAsDouble(M, 0, idx);
}

bool HasCvVector3Shape(const cv::Mat &M, const char *name)
{
    if(M.empty() || (M.rows < 3 && M.cols < 3))
    {
        std::cerr << "[Converter] invalid " << name << " shape: "
                  << M.rows << "x" << M.cols << std::endl;
        return false;
    }
    return true;
}

} // namespace

std::vector<cv::Mat> Converter::toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    if(!HasCvMatShape(cvT, 3, 4, "SE3 cvT"))
        return g2o::SE3Quat(Eigen::Matrix<double,3,3>::Identity(), Eigen::Matrix<double,3,1>::Zero());

    Eigen::Matrix<double,3,3> R;
    R << CvMatValueAsDouble(cvT,0,0), CvMatValueAsDouble(cvT,0,1), CvMatValueAsDouble(cvT,0,2),
         CvMatValueAsDouble(cvT,1,0), CvMatValueAsDouble(cvT,1,1), CvMatValueAsDouble(cvT,1,2),
         CvMatValueAsDouble(cvT,2,0), CvMatValueAsDouble(cvT,2,1), CvMatValueAsDouble(cvT,2,2);

    Eigen::Matrix<double,3,1> t(CvMatValueAsDouble(cvT,0,3), CvMatValueAsDouble(cvT,1,3), CvMatValueAsDouble(cvT,2,3));

    return g2o::SE3Quat(R,t);
}

g2o::SE3Quat Converter::toSE3Quat(const Sophus::SE3f &T)
{
    return g2o::SE3Quat(T.unit_quaternion().cast<double>(), T.translation().cast<double>());
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const g2o::Sim3 &Sim3)
{
    Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
    Eigen::Vector3d eigt = Sim3.translation();
    double s = Sim3.scale();
    return toCvSE3(s*eigR,eigt);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,4,4> &m)
{
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0;i<4;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,4> &m)
{
    cv::Mat cvMat(3,4,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<4; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3d &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix3f &m)
{
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::MatrixXf &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::MatrixXd &m)
{
    cv::Mat cvMat(m.rows(),m.cols(),CV_32F);
    for(int i=0;i<m.rows();i++)
        for(int j=0; j<m.cols(); j++)
            cvMat.at<float>(i,j)=m(i,j);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<float,3,1> &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
        cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

cv::Mat Converter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
{
    cv::Mat cvMat = cv::Mat::eye(4,4,CV_32F);
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cvMat.at<float>(i,j)=R(i,j);
        }
    }
    for(int i=0;i<3;i++)
    {
        cvMat.at<float>(i,3)=t(i);
    }

    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    if(!HasCvVector3Shape(cvVector, "Vector3d"))
        return Eigen::Matrix<double,3,1>::Zero();

    Eigen::Matrix<double,3,1> v;
    v << CvVectorValueAsDouble(cvVector, 0),
         CvVectorValueAsDouble(cvVector, 1),
         CvVectorValueAsDouble(cvVector, 2);

    return v;
}

Eigen::Matrix<float,3,1> Converter::toVector3f(const cv::Mat &cvVector)
{
    if(!HasCvVector3Shape(cvVector, "Vector3f"))
        return Eigen::Matrix<float,3,1>::Zero();

    Eigen::Matrix<float,3,1> v;
    v << static_cast<float>(CvVectorValueAsDouble(cvVector, 0)),
         static_cast<float>(CvVectorValueAsDouble(cvVector, 1)),
         static_cast<float>(CvVectorValueAsDouble(cvVector, 2));

    return v;
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Point3f &cvPoint)
{
    Eigen::Matrix<double,3,1> v;
    v << cvPoint.x, cvPoint.y, cvPoint.z;

    return v;
}

Eigen::Matrix<double,3,3> Converter::toMatrix3d(const cv::Mat &cvMat3)
{
    if(!HasCvMatShape(cvMat3, 3, 3, "Matrix3d"))
        return Eigen::Matrix<double,3,3>::Identity();

    Eigen::Matrix<double,3,3> M;

    M << CvMatValueAsDouble(cvMat3,0,0), CvMatValueAsDouble(cvMat3,0,1), CvMatValueAsDouble(cvMat3,0,2),
         CvMatValueAsDouble(cvMat3,1,0), CvMatValueAsDouble(cvMat3,1,1), CvMatValueAsDouble(cvMat3,1,2),
         CvMatValueAsDouble(cvMat3,2,0), CvMatValueAsDouble(cvMat3,2,1), CvMatValueAsDouble(cvMat3,2,2);

    return M;
}

Eigen::Matrix<double,4,4> Converter::toMatrix4d(const cv::Mat &cvMat4)
{
    if(!HasCvMatShape(cvMat4, 4, 4, "Matrix4d"))
        return Eigen::Matrix<double,4,4>::Identity();

    Eigen::Matrix<double,4,4> M;

    M << CvMatValueAsDouble(cvMat4,0,0), CvMatValueAsDouble(cvMat4,0,1), CvMatValueAsDouble(cvMat4,0,2), CvMatValueAsDouble(cvMat4,0,3),
         CvMatValueAsDouble(cvMat4,1,0), CvMatValueAsDouble(cvMat4,1,1), CvMatValueAsDouble(cvMat4,1,2), CvMatValueAsDouble(cvMat4,1,3),
         CvMatValueAsDouble(cvMat4,2,0), CvMatValueAsDouble(cvMat4,2,1), CvMatValueAsDouble(cvMat4,2,2), CvMatValueAsDouble(cvMat4,2,3),
         CvMatValueAsDouble(cvMat4,3,0), CvMatValueAsDouble(cvMat4,3,1), CvMatValueAsDouble(cvMat4,3,2), CvMatValueAsDouble(cvMat4,3,3);
    return M;
}

Eigen::Matrix<float,3,3> Converter::toMatrix3f(const cv::Mat &cvMat3)
{
    if(!HasCvMatShape(cvMat3, 3, 3, "Matrix3f"))
        return Eigen::Matrix<float,3,3>::Identity();

    Eigen::Matrix<float,3,3> M;

    M << static_cast<float>(CvMatValueAsDouble(cvMat3,0,0)), static_cast<float>(CvMatValueAsDouble(cvMat3,0,1)), static_cast<float>(CvMatValueAsDouble(cvMat3,0,2)),
         static_cast<float>(CvMatValueAsDouble(cvMat3,1,0)), static_cast<float>(CvMatValueAsDouble(cvMat3,1,1)), static_cast<float>(CvMatValueAsDouble(cvMat3,1,2)),
         static_cast<float>(CvMatValueAsDouble(cvMat3,2,0)), static_cast<float>(CvMatValueAsDouble(cvMat3,2,1)), static_cast<float>(CvMatValueAsDouble(cvMat3,2,2));

    return M;
}

Eigen::Matrix<float,4,4> Converter::toMatrix4f(const cv::Mat &cvMat4)
{
    if(!HasCvMatShape(cvMat4, 4, 4, "Matrix4f"))
        return Eigen::Matrix<float,4,4>::Identity();

    Eigen::Matrix<float,4,4> M;

    M << static_cast<float>(CvMatValueAsDouble(cvMat4,0,0)), static_cast<float>(CvMatValueAsDouble(cvMat4,0,1)), static_cast<float>(CvMatValueAsDouble(cvMat4,0,2)), static_cast<float>(CvMatValueAsDouble(cvMat4,0,3)),
         static_cast<float>(CvMatValueAsDouble(cvMat4,1,0)), static_cast<float>(CvMatValueAsDouble(cvMat4,1,1)), static_cast<float>(CvMatValueAsDouble(cvMat4,1,2)), static_cast<float>(CvMatValueAsDouble(cvMat4,1,3)),
         static_cast<float>(CvMatValueAsDouble(cvMat4,2,0)), static_cast<float>(CvMatValueAsDouble(cvMat4,2,1)), static_cast<float>(CvMatValueAsDouble(cvMat4,2,2)), static_cast<float>(CvMatValueAsDouble(cvMat4,2,3)),
         static_cast<float>(CvMatValueAsDouble(cvMat4,3,0)), static_cast<float>(CvMatValueAsDouble(cvMat4,3,1)), static_cast<float>(CvMatValueAsDouble(cvMat4,3,2)), static_cast<float>(CvMatValueAsDouble(cvMat4,3,3));
    return M;
}

std::vector<float> Converter::toQuaternion(const cv::Mat &M)
{
    Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
    Eigen::Quaterniond q(eigMat);

    std::vector<float> v(4);
    v[0] = q.x();
    v[1] = q.y();
    v[2] = q.z();
    v[3] = q.w();

    return v;
}

cv::Mat Converter::tocvSkewMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

bool Converter::isRotationMatrix(const cv::Mat &R)
{
    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}

std::vector<float> Converter::toEuler(const cv::Mat &R)
{
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<float>(0,0) * R.at<float>(0,0) +  R.at<float>(1,0) * R.at<float>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<float>(2,1) , R.at<float>(2,2));
        y = atan2(-R.at<float>(2,0), sy);
        z = atan2(R.at<float>(1,0), R.at<float>(0,0));
    }
    else
    {
        x = atan2(-R.at<float>(1,2), R.at<float>(1,1));
        y = atan2(-R.at<float>(2,0), sy);
        z = 0;
    }

    std::vector<float> v_euler(3);
    v_euler[0] = x;
    v_euler[1] = y;
    v_euler[2] = z;

    return v_euler;
}

Sophus::SE3<float> Converter::toSophus(const cv::Mat &T) {
    if(!HasCvMatShape(T, 3, 4, "Sophus SE3"))
        return Sophus::SE3<float>();

    Eigen::Matrix<double,3,3> eigMat;
    eigMat << CvMatValueAsDouble(T,0,0), CvMatValueAsDouble(T,0,1), CvMatValueAsDouble(T,0,2),
              CvMatValueAsDouble(T,1,0), CvMatValueAsDouble(T,1,1), CvMatValueAsDouble(T,1,2),
              CvMatValueAsDouble(T,2,0), CvMatValueAsDouble(T,2,1), CvMatValueAsDouble(T,2,2);
    Eigen::Quaternionf q(eigMat.cast<float>());

    Eigen::Matrix<float,3,1> t;
    t << static_cast<float>(CvMatValueAsDouble(T,0,3)),
         static_cast<float>(CvMatValueAsDouble(T,1,3)),
         static_cast<float>(CvMatValueAsDouble(T,2,3));

    return Sophus::SE3<float>(q,t);
}

Sophus::Sim3f Converter::toSophus(const g2o::Sim3& S) {
    return Sophus::Sim3f(Sophus::RxSO3d((float)S.scale(), S.rotation().matrix()).cast<float>() ,
                         S.translation().cast<float>());
}

} //namespace ORB_SLAM
