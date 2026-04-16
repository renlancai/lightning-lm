#pragma once

#include "core/graph/base_unary_edge.h"
#include "core/types/vertex_se3.h"

namespace lightning::miao {

/**
 * 3D 位置先验约束（仅约束平移，旋转自由）
 *
 * 误差：e = T.translation - measurement (3×1)
 * 雅可比（对 VertexSE3 的 6D 扰动 [dt, dR]）：
 *   J = [I_3x3 | 0_3x3]  (3×6)
 *
 * 用于 RTK/GNSS 绝对位置约束，不约束朝向。
 */
class EdgePositionPrior : public BaseUnaryEdge<3, Vec3d, VertexSE3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void ComputeError() override {
        SE3 pose = ((VertexSE3*)(vertices_[0]))->Estimate();
        error_ = pose.translation() - measurement_;
    }

    void LinearizeOplus() override {
        // update = [dt (3), dR (3)]; translation DOF first
        jacobian_oplus_xi_.setZero();
        jacobian_oplus_xi_.template block<3, 3>(0, 0).setIdentity();
    }
};

}  // namespace lightning::miao
