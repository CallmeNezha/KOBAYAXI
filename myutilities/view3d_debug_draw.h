#ifndef VIEW3D_DEBUG_DRAW_H_HEADER_GUARD
#define VIEW3D_DEBUG_DRAW_H_HEADER_GUARD
#include "bgfx.h"
#include <eigen/core>
#include <opencv2/opencv.hpp>

namespace VIEW3D
{
    ///
    void drawObb(
        float _sx, float _sy, float _sz
        , float _ax, float _ay, float _az
        , float _tx, float _ty, float _tz, uint32_t _color, bool _wireframe);
    void drawObb(const float* _mtx, float _sx, float _sy, float _sz, uint32_t _color, bool _wireframe);

    void drawHMDBox(const float* _mtx, uint32_t _color, bool _wireframe);
    void drawHandle(const float* _mtx, uint32_t _color, bool _wireframe);
    void drawCamera(const float* _mtx, uint32_t _color);
    void drawAxis  (const float* _mtx, float _length);


    ///
    inline void Eigen2Array(const Eigen::Matrix4f& _src, float* _dst)
    {
        bx::memCopy(_dst, _src.data(), 16 * sizeof(float));
    }

    inline void Array2Eigen(const float* _src, Eigen::Matrix4f& _dst)
    {
        bx::memCopy(_dst.data(), _src, 16 * sizeof(float));
    }

    inline Eigen::Vector3f getFirst3(const Eigen::Vector4f& _vec)
    {
        return Eigen::Vector3f(_vec[0], _vec[1], _vec[2]);
    }

    inline Eigen::Vector3f getPosition(const Eigen::Matrix4f& _mtx)
    {
        return _mtx.block(0, 3, 3, 1);
    }

    inline Eigen::Matrix3f getRotation(const Eigen::Matrix4f& _mtx)
    {
        return _mtx.block(0, 0, 3, 3);
    }

    inline Eigen::Vector3f getX(const Eigen::Matrix4f& _mtx)
    {
        return getFirst3(_mtx * Eigen::Vector4f(1.f, 0.f, 0.f, 1.f));
    }

    inline Eigen::Vector3f getY(const Eigen::Matrix4f& _mtx)
    {
        return getFirst3(_mtx * Eigen::Vector4f(0.f, 1.f, 0.f, 1.f));
    }

    inline Eigen::Vector3f getZ(const Eigen::Matrix4f& _mtx)
    {
        return getFirst3(_mtx * Eigen::Vector4f(0.f, 0.f, 1.f, 1.f));
    }


    /// Rotation matrix to euler angles.
    // https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    // http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
    inline std::pair<Eigen::Vector3f, Eigen::Vector3f> mtxDecompose(const Eigen::Matrix4f& _mtx)
    {
        Eigen::Matrix4f mtx = _mtx; mtx = mtx.transpose();
        Eigen::Matrix3f R = mtx.block(0, 0, 3, 3);
        bool bOrtho = (Eigen::Matrix3f::Identity() - R.transpose() * R).norm() < 1e-6;
        BX_CHECK(bOrtho, "Matrix is not a transform.")
            Eigen::Vector3f T = mtx.block(0, 3, 3, 1);
        float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        bool bSingular = sy < 1e-6;

        float x, y, z;
        if (!bSingular)
        {
            x = atan2(R(2, 1), R(2, 2));
            y = atan2(-R(2, 0), sy);
            z = atan2(R(1, 0), R(0, 0));
        }
        else
        {
            x = atan2(-R(1, 2), R(1, 1));
            y = atan2(-R(2, 0), sy);
            z = 0.f;
        }
        return std::make_pair(Eigen::Vector3f(x, y, z), T);
    }

    // Calculates rotation matrix given euler angles.
    inline cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f &theta)
    {
        // Calculate rotation about x axis
        cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
            1, 0, 0,
            0, cos(theta[0]), -sin(theta[0]),
            0, sin(theta[0]), cos(theta[0])
            );

        // Calculate rotation about y axis
        cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
            cos(theta[1]), 0, sin(theta[1]),
            0, 1, 0,
            -sin(theta[1]), 0, cos(theta[1])
            );

        // Calculate rotation about z axis
        cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
            cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]), cos(theta[2]), 0,
            0, 0, 1);


        // Combined rotation matrix
        cv::Mat R = R_z * R_y * R_x;

        return R;

    }

}



#endif // !VIEW3D_DEBUG_DRAW_H_HEADER_GUARD