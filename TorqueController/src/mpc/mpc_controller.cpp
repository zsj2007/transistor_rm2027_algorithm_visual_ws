#include "mpc/mpc_controller.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <ceres/ceres.h>

// ------------------------------------------------------------
// Ceres 代价函数（增量重参数化：d[0]=u[0]，d[k]=u[k]-u[k-1]）
// 模板化 operator()：T = ceres::Jet 时由 DynamicAutoDiffCostFunction
// 通过链式法则自动求导（动力学经 predictTrajectory 模板传播梯度）。
// ------------------------------------------------------------
class MPCCostFunctor {
public:
    MPCCostFunctor(const MPCController* mpc, double theta0, double omega0,
                   const std::vector<double>& theta_ref)
        : mpc_(mpc), theta0_(theta0), omega0_(omega0), theta_ref_(theta_ref) {}

    template <typename T>
    bool operator()(T const* const* parameters, T* residuals) const {
        const T* d = parameters[0];
        const int N = mpc_->N();

        // 由增量 d 重建力矩序列 u（第一步相对上次实际力矩，含最大力矩硬限幅）
        const T tau_max = T(mpc_->maxTorque());
        std::vector<T> u(N);
        u[0] = std::clamp(mpc_->prevTorque() + d[0], -tau_max, tau_max);
        for (int k = 1; k < N; ++k) {
            T u_k = u[k - 1] + d[k];
            u[k] = std::clamp(u_k, -tau_max, tau_max);
        }

        std::vector<T> theta_pred, omega_pred;
        mpc_->predictTrajectory(T(theta0_), T(omega0_), u, theta_pred, omega_pred);

        int idx = 0;
        // 位置跟踪误差：直接相减，不归一化（多圈连续语义，
        // theta_pred[k] 为 k+1 个控制周期后的预测，ref[k] 对应参考）
        for (int k = 0; k < N; ++k) {
            T err = theta_pred[k] - theta_ref_[k];
            residuals[idx++] = std::sqrt(mpc_->Q()) * err;
        }
        // 控制量惩罚
        for (int k = 0; k < N; ++k) {
            residuals[idx++] = std::sqrt(mpc_->R()) * u[k];
        }
        // 力矩变化率惩罚（软约束，硬约束由 d[k] 的参数边界保证）
        for (int k = 1; k < N; ++k) {
            residuals[idx++] = std::sqrt(mpc_->Rd()) * d[k];
        }
        return true;
    }

private:
    const MPCController* mpc_;
    double theta0_, omega0_;
    std::vector<double> theta_ref_;
};

// ------------------------------------------------------------
// 构造函数
// ------------------------------------------------------------
MPCController::MPCController(double dt_control, double dt_sim,
                             double J, double tau_c, double b, double tau_d,
                             double max_torque, double max_torque_rate,
                             int N, double Q, double R, double Rd, int max_iter)
    : dt_control_(dt_control), dt_sim_(dt_sim),
      J_(J), tau_c_(tau_c), b_(b), tau_d_(tau_d),
      max_torque_(max_torque), max_torque_rate_(max_torque_rate),
      N_(N), Q_(Q), R_(R), Rd_(Rd), max_iter_(max_iter)
{
    double ratio = dt_control_ / dt_sim_;
    steps_per_control_ = static_cast<int>(std::round(ratio));
    if (std::abs(steps_per_control_ * dt_sim_ - dt_control_) > 1e-9) {
        throw std::invalid_argument("dt_control must be an integer multiple of dt_sim");
    }
    rate_step_ = max_torque_rate_ * dt_control_;
    prev_u_seq_.assign(N_, 0.0);
}

// ------------------------------------------------------------
// 核心 step 方法（同步求解，返回第一步控制量）
// ------------------------------------------------------------
MPCController::Result MPCController::step(double theta, double omega,
                                          const std::vector<double>& theta_ref) {
    if (theta_ref.size() < static_cast<size_t>(N_)) {
        throw std::invalid_argument("theta_ref length must be at least N");
    }
    Result result;
    std::vector<double> ref_copy = theta_ref;

    // 初始猜测：平移上一次最优序列 -> 转为增量 d
    std::vector<double> u_init(N_, 0.0);
    for (int i = 0; i < N_ - 1; ++i) u_init[i] = prev_u_seq_[i + 1];
    if (!prev_u_seq_.empty()) u_init[N_ - 1] = prev_u_seq_.back();

    std::vector<double> d(N_, 0.0);
    d[0] = std::clamp(u_init[0] - prev_torque_, -rate_step_, rate_step_);
    for (int k = 1; k < N_; ++k) {
        d[k] = std::clamp(u_init[k] - u_init[k - 1], -rate_step_, rate_step_);
    }

    ceres::Problem problem;
    MPCCostFunctor* cost_functor = new MPCCostFunctor(this, theta, omega, ref_copy);
    // 残差 = 位置误差(N) + 控制量惩罚(N) + 力矩变化率惩罚(N-1)
    int num_residuals = N_ + N_ + (N_ - 1);
    // 自动求导：动力学模板化，Ceres 用 Jet 链式法则计算雅可比（替代数值微分）
    auto* cost_function = new ceres::DynamicAutoDiffCostFunction<MPCCostFunctor>(
        cost_functor, ceres::TAKE_OWNERSHIP);
    cost_function->SetNumResiduals(num_residuals);
    cost_function->AddParameterBlock(N_);
    problem.AddResidualBlock(cost_function, nullptr, d.data());

    // 参数硬边界：所有增量 d[k] 均受最大力矩变化率限制
    for (int k = 0; k < N_; ++k) {
        problem.SetParameterLowerBound(d.data(), k, -rate_step_);
        problem.SetParameterUpperBound(d.data(), k,  rate_step_);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = max_iter_;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 由最终 d 重建力矩序列
    std::vector<double> u(N_);
    u[0] = std::clamp(prev_torque_ + d[0], -max_torque_, max_torque_);
    for (int k = 1; k < N_; ++k) {
        u[k] = std::clamp(u[k - 1] + d[k], -max_torque_, max_torque_);
    }

    if (!summary.IsSolutionUsable()) {
        // 求解失败：退化为 0 力矩，输出当前状态
        prev_u_seq_.assign(N_, 0.0);
        prev_torque_ = 0.0;
        result.torque = 0.0;
        result.theta = theta;
        result.omega = omega;
        return result;
    }

    prev_u_seq_ = u;
    prev_torque_ = u[0];

    // 缓存本次参考序列与完整预测位置序列（供上层查询）
    last_ref_ = ref_copy;
    last_pred_.clear();
    {
        std::vector<double> theta_pred, omega_pred;
        predictTrajectory(theta, omega, u, theta_pred, omega_pred);   // double 实例化
        // predictTrajectory 输出 N 个预测点（不含当前），与 last_ref_ 一一对应
        last_pred_ = theta_pred;
    }

    // 第一步预测状态（应用 u[0] 一个控制周期）
    double th = theta, om = omega;
    for (int step = 0; step < steps_per_control_; ++step) {
        auto p = dynamicsStep(th, om, u[0], dt_sim_);
        th = p.first;
        om = p.second;
    }
    result.torque = u[0];
    result.theta = th;
    result.omega = om;
    return result;
}
