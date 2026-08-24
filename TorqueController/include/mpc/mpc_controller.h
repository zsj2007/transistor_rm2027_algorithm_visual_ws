#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

// ============================================================================
// MPCController — 偏航轴 MPC 求解器（Ceres）
//
// 模型: J * dω/dt = τ - τ_c * sign(ω) - b * ω + τ_d
//   （sign 用 tanh 软符号近似，与 param_ident 一致）
//
// 限制:
//   - 最大力矩   max_torque      (N·m)，参数硬边界
//   - 最大力矩变化率 max_torque_rate (N·m/s)，通过增量重参数化硬约束
//
// 参考轨迹: theta_ref 为未来 N 个控制周期（长度 N）的多圈连续参考角度，
//   ref[i] 对应预测 theta_pred[i+1]；位置误差直接相减（不归一化到 (-π, π]），
//   保留圈数语义——目标与当前状态差多少圈就跟踪多少圈。
//
// 求导方式: 动力学/代价函数均模板化（T = double 或 ceres::Jet），
//   由 Ceres DynamicAutoDiffCostFunction 自动求导（链式法则），
//   替代原先的有限差分数值微分。
// ============================================================================
class MPCController {
public:
    struct Result {
        double torque = 0.0;  // 第一步控制力矩
        double theta  = 0.0;  // 第一步预测位置 (rad)
        double omega  = 0.0;  // 第一步预测速度 (rad/s)
    };

    MPCController(double dt_control, double dt_sim,
                  double J, double tau_c, double b, double tau_d,
                  double max_torque, double max_torque_rate,
                  int N, double Q, double R, double Rd, int max_iter);

    Result step(double theta, double omega, const std::vector<double>& theta_ref);

    // ── 供 CostFunctor 调用（模板：double 运行时 / ceres::Jet 自动求导）──
    // 输出 N 个预测点：theta_pred[k] / omega_pred[k] 为施加 u[0..k] 后
    // （k+1 个控制周期后）的状态，不含初始状态 theta0
    template <typename T>
    void predictTrajectory(const T& theta0, const T& omega0,
                           const std::vector<T>& u_seq,
                           std::vector<T>& theta_pred,
                           std::vector<T>& omega_pred) const {
        theta_pred.clear();
        omega_pred.clear();
        theta_pred.reserve(N_);
        omega_pred.reserve(N_);

        T theta = theta0, omega = omega0;
        for (int k = 0; k < N_; ++k) {
            T tau = u_seq[k];
            for (int step = 0; step < steps_per_control_; ++step) {
                auto p = dynamicsStep(theta, omega, tau, dt_sim_);
                theta = p.first;
                omega = p.second;
            }
            theta_pred.push_back(theta);
            omega_pred.push_back(omega);
        }
    }

    // 供 CostFunctor 读取
    int    N()          const { return N_; }
    double Q()          const { return Q_; }
    double R()          const { return R_; }
    double Rd()         const { return Rd_; }
    double maxTorque()  const { return max_torque_; }
    double rateStep()   const { return rate_step_; }
    double prevTorque() const { return prev_torque_; }

    // 最新一次 step 的参考（目标）序列（N 个）与预测位置序列（N 个，不含当前状态）
    const std::vector<double>& lastRef()  const { return last_ref_; }
    const std::vector<double>& lastPred() const { return last_pred_; }

private:
    double dt_control_, dt_sim_;
    int    steps_per_control_;
    double J_, tau_c_, b_, tau_d_;
    double max_torque_, max_torque_rate_;
    double rate_step_;   // 每个控制步允许的最大力矩增量 = max_torque_rate * dt_control
    int    N_;
    double Q_, R_, Rd_;
    int    max_iter_;

    // 上一次最优控制序列（用于热启动）
    std::vector<double> prev_u_seq_;
    // 上一次实际施加的力矩（用于第一步力矩变化率硬约束）
    double prev_torque_ = 0.0;

    // 最新一次 step 的参考序列与预测位置序列（供上层查询）
    std::vector<double> last_ref_;
    std::vector<double> last_pred_;

    // 摩擦力矩（软符号 tanh；tanh 依赖 ADL：double → ::tanh，Jet → ceres::tanh）
    template <typename T>
    T frictionTorque(const T& omega) const {
        const double lambda_omega = 100.0;
        T soft_sign = tanh(lambda_omega * omega);
        return -soft_sign * tau_c_ - b_ * omega;
    }

    // 单步显式欧拉动力学（不对速度做任何截断/限制）
    template <typename T>
    std::pair<T, T> dynamicsStep(const T& theta, const T& omega,
                                 const T& tau, double dt) const {
        T tau_f = frictionTorque(omega);
        T tau_net = tau + tau_f + tau_d_;
        T alpha = tau_net / J_;
        T omega_new = omega + alpha * dt;
        T theta_new = theta + omega_new * dt;
        return {theta_new, omega_new};
    }
};

#endif // MPC_CONTROLLER_H
