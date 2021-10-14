//
// Created by 闻永言 on 2021/10/7.
//

#include "Correction.hpp"

#include "Type.hpp"
#include "Eigen/Geometry"
#include "ceres/ceres.h"

struct ReProjectionError
{
    const Eigen::Matrix4d PV; // view space to screen space transform matrix, VPV = M_viewport * M_per
    const Eigen::Vector4d point_3d;
    const double x;
    const double y;

    ReProjectionError(const float4x4 &M_per, const float4x4 &M_viewport, const float4 &_point_3d, double _x, double _y):PV(
            (M_viewport * M_per).cast<double>()), point_3d(_point_3d.cast<double>()), x(_x), y(_y){}

    template<typename T> bool operator()(const T *const view, T *residual) const
    {
        Eigen::Matrix<T, 4, 4> M_view;
        M_view
                << view[0], view[1], view[2], view[9], view[3], view[4], view[5], view[10], view[6], view[7], view[8], view[11], T(
                0), T(0), T(0), T(1.0);
        Eigen::Matrix<T, 4, 1> refer_2d = PV.template cast<T>() * M_view * point_3d.template cast<T>();
        refer_2d = refer_2d / refer_2d.w();
        T recon_x = refer_2d[0];
        T recon_y = refer_2d[1];

        residual[0] = recon_x - T(x);
        residual[1] = recon_y - T(y);
        return true;
    }

    static ceres::CostFunction *
    create(const float4x4 &M_per, const float4x4 &M_viewport, const float4 &_point_3d, double _x, double _y)
    {
        return new ceres::AutoDiffCostFunction<ReProjectionError, 2, 12>(
                new ReProjectionError(M_per, M_viewport, _point_3d, _x, _y));
    }
};

struct ReProjectionErrorEulerAngle
{
    const Eigen::Matrix4d PV; // view space to screen space transform matrix, VPV = M_viewport * M_per
    const Eigen::Vector4d point_3d;
    const double x;
    const double y;

    ReProjectionErrorEulerAngle(const float4x4 &M_per, const float4x4 &M_viewport, const float4 &_point_3d, double _x, double _y)
            :PV((M_viewport * M_per).cast<double>()), point_3d(_point_3d.cast<double>()), x(_x), y(_y){}

    template<typename T> bool operator()(const T *const view, T *residual) const
    {
        Eigen::Matrix<T, 4, 4> M_view;
        Eigen::Matrix<T, 3, 3> R;
        R = Eigen::AngleAxis<T>(view[2], Eigen::Vector<T, 3>::UnitX()) *
            Eigen::AngleAxis<T>(view[1], Eigen::Vector<T, 3>::UnitY()) *
            Eigen::AngleAxis<T>(view[0], Eigen::Vector<T, 3>::UnitZ());

        M_view << R, Eigen::Vector<T, 3>(view[2], view[1], view[0]), Eigen::RowVector<T, 4>(T(0), T(0), T(0), T(1.0));
//        M_view(0, 0) = R(0, 0);
//        M_view(0, 1) = R(0, 1);
//        M_view(0, 2) = R(0, 2);
//        M_view(1, 0) = R(1, 0);
//        M_view(1, 1) = R(1, 1);
//        M_view(1, 2) = R(1, 2);
//        M_view(2, 0) = R(2, 0);
//        M_view(2, 1) = R(2, 1);
//        M_view(2, 2) = R(2, 2);
//        M_view(0, 3) = view[3];
//        M_view(1, 3) = view[4];
//        M_view(2, 3) = view[5];
//        M_view(3, 3) = T(1.0);

        Eigen::Matrix<T, 4, 1> refer_2d = PV.template cast<T>() * M_view * point_3d.template cast<T>();
        refer_2d = refer_2d / refer_2d.w();
        T recon_x = refer_2d[0];
        T recon_y = refer_2d[1];

        residual[0] = recon_x - T(x);
        residual[1] = recon_y - T(y);
        return true;
    }

    static ceres::CostFunction *
    create(const float4x4 &M_per, const float4x4 &M_viewport, const float4 &_point_3d, double _x, double _y)
    {
        return new ceres::AutoDiffCostFunction<ReProjectionErrorEulerAngle, 2, 6>(
                new ReProjectionErrorEulerAngle(M_per, M_viewport, _point_3d, _x, _y));
    }
};

void Correction::correction(
        Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
        const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices, SfS::CORRECTION_MODE mode)
{
    switch (mode)
    {
        case SfS::MATRIX:
            correction_matrix(uniform, matches_3d_2d, matches_2d_2d, vertices);
            break;
        case SfS::EULER_ANGLE:
            correction_euler(uniform, matches_3d_2d, matches_2d_2d, vertices);
            break;
    }
}

void Correction::correction_matrix(
        Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
        const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices)
{
    index_t image_x = uniform->refer_image->x;
    index_t image_y = uniform->refer_image->y;
    index_t image_size = image_x * image_y;

    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        ceres::Problem problem;
        const float4x4 &M_viewport = uniform->cameras->at(i)->M_viewport;
        const float4x4 &M_per = uniform->cameras->at(i)->M_per;
        float4x4 &M_view = uniform->cameras->at(i)->M_view;

        double view[12] =
                {M_view(0, 0), M_view(0, 1), M_view(0, 2), M_view(1, 0), M_view(1, 1), M_view(1, 2), M_view(2, 0),
                 M_view(2, 1), M_view(2, 2), M_view(0, 3), M_view(1, 3), M_view(2, 3)};
        cout << "camera " << i << " (before correction): " << endl;
        cout << view[0] << " " << view[1] << " " << view[2] << " " << view[3] << " " << view[4] << " " << view[5] << " "
             << view[6] << " " << view[7] << " " << view[8] << " " << view[9] << " " << view[10] << " " << view[11]
             << endl;
        for (const auto &p1: matches_3d_2d)
        {
            index_t refer_index;
            index_t start = i * image_size;
            index_t end = start + image_size;
            for (const auto &p2: matches_2d_2d)
            {
                if (p2.second >= start && p2.second < end && p2.first == p1.second)
                {
                    refer_index = p2.second;
                    refer_index -= start;
                    index_t point_x = refer_index % image_x;
                    index_t point_y = refer_index / image_x;

                    //                    cout << point_x << " " << point_y << endl;
                    ceres::CostFunction *cost_func = ReProjectionError::create(
                            M_per, M_viewport, vertices[p1.first].world, (double)point_x + 0.5, (double)point_y + 0.5);
                    problem.AddResidualBlock(cost_func, nullptr, view);
                    break;
                }
            }
        }

        ceres::Solver::Options solver_options;
        //        solver_options.linear_solver_type = ceres::DENSE_QR;
        solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        //        solver_options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        //        cout << "summary:\n" << summary.BriefReport() << endl;
        M_view
                << view[0], view[1], view[2], view[9], view[3], view[4], view[5], view[10], view[6], view[7], view[8], view[11], 0, 0, 0, 1.0f;

        // update
        uniform->cameras->at(i)->P = M_viewport * M_per * M_view;
        cout << "camera " << i << " (after correction): " << endl;
        cout << view[0] << " " << view[1] << " " << view[2] << " " << view[3] << " " << view[4] << " " << view[5] << " "
             << view[6] << " " << view[7] << " " << view[8] << " " << view[9] << " " << view[10] << " " << view[11]
             << endl;
        //        for (const auto &p1: matches_3d_2d)
        //        {
        //            index_t refer_index;
        //            index_t start = i * image_size;
        //            index_t end = start + image_size;
        //            for (const auto &p2: matches_2d_2d)
        //            {
        //                if (p2.second >= start && p2.second < end && p2.first == p1.second)
        //                {
        //                    vec4 refine = M_viewport * M_per * M_view * vertices[p1.first].world;
        //                    refine /= refine.w();
        //                    cout << refine.x() << " " << refine.y() << endl;
        //                    break;
        //                }
        //            }
        //        }
        //return summary.IsSolutionUsable();
    }
}

void Correction::correction_euler(
        Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
        const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices)
{
    index_t image_x = uniform->refer_image->x;
    index_t image_y = uniform->refer_image->y;
    index_t image_size = image_x * image_y;

    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        ceres::Problem problem;
        const float4x4 &M_viewport = uniform->cameras->at(i)->M_viewport;
        const float4x4 &M_per = uniform->cameras->at(i)->M_per;
        float4x4 &M_view = uniform->cameras->at(i)->M_view;

        float3x3 R;
        R << M_view(0, 0), M_view(0, 1), M_view(0, 2),
                M_view(1, 0), M_view(1, 1), M_view(1, 2),
                M_view(2, 0), M_view(2, 1), M_view(2, 2);
        float3 euler = R.eulerAngles(2, 1, 0);
        double view[6] = {euler[0], euler[1], euler[2], M_view(0, 3), M_view(1, 3), M_view(2, 3)};
        cout << "camera " << i << " (before correction): " << endl;
        cout << view[0] << " " << view[1] << " " << view[2] << " " << view[3] << " " << view[4] << " " << view[5]
             << endl;
        for (const auto &p1: matches_3d_2d)
        {
            index_t refer_index;
            index_t start = i * image_size;
            index_t end = start + image_size;
            for (const auto &p2: matches_2d_2d)
            {
                if (p2.second >= start && p2.second < end && p2.first == p1.second)
                {
                    refer_index = p2.second;
                    refer_index -= start;
                    index_t point_x = refer_index % image_x;
                    index_t point_y = refer_index / image_x;

                    //                    cout << point_x << " " << point_y << endl;
                    ceres::CostFunction *cost_func = ReProjectionErrorEulerAngle::create(
                            M_per, M_viewport, vertices[p1.first].world, (double)point_x + 0.5, (double)point_y + 0.5);
                    problem.AddResidualBlock(cost_func, nullptr, view);
                    break;
                }
            }
        }

        ceres::Solver::Options solver_options;
        //        solver_options.linear_solver_type = ceres::DENSE_QR;
        solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        //        solver_options.minimizer_progress_to_stdout = true;
        ceres::Solver::Summary summary;
        ceres::Solve(solver_options, &problem, &summary);
        //        cout << "summary:\n" << summary.BriefReport() << endl;

        R = Eigen::AngleAxisf(view[2], float3::UnitX()) * Eigen::AngleAxisf(view[1], float3::UnitY()) *
            Eigen::AngleAxisf(view[0], float3::UnitZ());
        M_view(0, 0) = R(0, 0);
        M_view(0, 1) = R(0, 1);
        M_view(0, 2) = R(0, 2);
        M_view(1, 0) = R(1, 0);
        M_view(1, 1) = R(1, 1);
        M_view(1, 2) = R(1, 2);
        M_view(2, 0) = R(2, 0);
        M_view(2, 1) = R(2, 1);
        M_view(2, 2) = R(2, 2);
        M_view(0, 3) = view[3];
        M_view(1, 3) = view[4];
        M_view(2, 3) = view[5];

        // update
        uniform->cameras->at(i)->P = M_viewport * M_per * M_view;
        cout << "camera " << i << " (after correction): " << endl;
        cout << view[0] << " " << view[1] << " " << view[2] << " " << view[3] << " " << view[4] << " " << view[5]
             << endl;
    }
}
