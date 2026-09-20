#ifndef _TRAJ_OPTIMIZER_H_
#define _TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <string>

#include <plan_utils/traj_container.hpp>

#include "geo_utils2d/lbfgs.hpp"
#include "geo_utils2d/geoutils2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
namespace plan_manage
{

  using namespace std;

  // MINCO 核心的显式配置。调用方无需依赖 ROS 参数服务器。
  struct MincoConfig
  {
    int traj_resolution = 8;
    int destraj_resolution = 20;
    double wei_obs = 7000.0;
    double wei_surround = 7000.0;
    double wei_feas = 1000.0;
    double wei_time = 500.0;
    double wei_anchor = 0.0;
    double surround_clearance = 1.0;
    double max_vel = 3.0;
    double max_acc = 1.5;
    double max_cur = 0.523598;
    double half_margin = 0.25;
    int cars_num = 1;
    int car_id = 0;
    double car_length = 0.6;
    double car_width = 0.6;
    double car_d_cr = 0.0;
    // 大于0时每隔该次数输出一次代价分解；0表示关闭迭代日志。
    int logging_every_n = 50;
  };

  // 单条 MINCO 轨迹的输入。状态矩阵按 [位置, 速度, 加速度] 组织，维度为 2x3。
  struct MincoRequest
  {
    Eigen::MatrixXd initial_state;
    Eigen::MatrixXd final_state;
    Eigen::MatrixXd initial_inner_points;
    double total_time = 0.0;
    // 默认沿用凸安全走廊约束；关闭时只计算关联静态障碍点的距离代价。
    bool use_safe_corridor_constraints = true;
    // 仅在关闭安全走廊时使用；false表示不计算关联静态障碍点距离代价。
    bool use_associated_obstacle_constraints = false;
    std::vector<Eigen::MatrixXd> sampled_corridors;
    // 与约束采样点一一对应；每项保存该采样点关联的占据栅格中心。
    std::vector<std::vector<Eigen::Vector2d>> associated_obstacle_points;
    double static_obstacle_clearance = 0.0;
    Eigen::VectorXd piece_time_ratios;
    int singular_direction = 1;
    double start_time = 0.0;
    double curvature_epsilon = 1.0e-4;
    std::function<bool()> cancel_checker;
  };

  struct MincoResult
  {
    bool success = false;
    bool cancelled = false;
    std::string failure_reason;
    int iterations = 0;
    double optimize_time_ms = 0.0;
    double final_cost = 0.0;
    int solver_result = 0;
    plan_utils::Trajectory trajectory;
  };


  class PolyTrajOptimizer
  {

  private:


    plan_utils::SurroundTrajData *surround_trajs_{NULL}; // Can not use shared_ptr and no need to free
    std::vector<plan_utils::TrajContainer> swarm_traj_container_;
    std::vector<plan_utils::TrajContainer> swarm_last_traj_container_;
    bool ifdynamic_;
    int car_id_, cars_num_;
    int traj_resolution_; // number of distinctive constrain points each piece
    int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
    int variable_num_;     // optimization variables
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in surround

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_surround_;                       // surround weight
    double wei_feas_;                        // feasibility weight
    double wei_time_;                        // time weight
    double wei_anchor_;                      // RDP内部关键点锚定权重
    double surround_clearance_; // safe distance
    double max_vel_, max_acc_, max_cur_;       // dynamic limits
    double half_margin;                        // safe margin
    // common::VehicleParam veh_param_;    
    double car_length_, car_width_, car_d_cr_;


    double t_now_;
    double delta_t_;// 0.2
    double L_, max_phidot_;
    std::vector<Eigen::Vector2d> lz_set_;
    std::vector<Eigen::Vector2d> vec_le_, vec_lo_;
    int number_of_hyperplanes_of_ego_car_, number_of_hyperplanes_of_surround_car_;
    std::vector<bool> have_received_trajs_;
    // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
    Eigen::Matrix<double, 2, 2> B_h;
    double epis; // epis = 0.0
    std::vector<int> singul_container;
    std::vector<Eigen::MatrixXd> iniState_container;
    std::vector<Eigen::MatrixXd> finState_container;
    std::vector<Eigen::MatrixXd> anchor_points_container;
    std::vector<std::vector<Eigen::MatrixXd>> cfgHs_container;
    bool use_safe_corridor_constraints_ = true;
    bool use_associated_obstacle_constraints_ = false;
    std::vector<std::vector<std::vector<Eigen::Vector2d>>> associated_obstacle_points_container_;
    double static_obstacle_clearance_ = 0.0;
    std::vector<Eigen::VectorXd> piece_time_ratios_container;
    int trajnum;//轨迹只有一条，所以trajnum = 1
    std::vector<plan_utils::MinJerkOpt> jerkOpt_container;
    std::vector<int> piece_num_container;
    std::function<bool()> cancel_checker_;
    bool last_optimization_cancelled_ = false;
    bool verbose_ = true;
    int logging_every_n_ = 50;
    int last_iteration_count_ = 0;
    double last_optimize_time_ms_ = 0.0;
    double last_final_cost_ = 0.0;
    int last_solver_result_ = 0;

  public:
    
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* set variables */
    void init(ros::NodeHandle &nh);
    void configure(const MincoConfig &config);
    void setSurroundTrajs(plan_utils::SurroundTrajData *surround_trajs_ptr);
    void setSwarmTrajs(std::vector<plan_utils::TrajContainer> &swarm_traj_container, bool ifdynamic);
    void setAllCarsTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    void setAllCarsLastTrajs(plan_utils::TrajContainer& trajectory, int& car_id);
    bool checkCollisionWithSurroundCars(const double& time);

    /* helper functions */
    inline const std::vector<plan_utils::MinJerkOpt> *getMinJerkOptPtr(void) { return &jerkOpt_container; }
    inline int get_traj_resolution_() { return traj_resolution_; };
    inline int get_destraj_resolution_() { return destraj_resolution_; };
    inline double getsurroundClearance(void) { return surround_clearance_; }
    void setCancelChecker(const std::function<bool()>& cancel_checker)
    {
      cancel_checker_ = cancel_checker;
    }
    inline bool wasLastOptimizationCancelled() const
    {
      return last_optimization_cancelled_;
    }
    void setVerbose(const bool verbose) { verbose_ = verbose; }
    inline int lastIterationCount() const { return last_iteration_count_; }
    inline double lastOptimizeTimeMs() const { return last_optimize_time_ms_; }
    inline double lastFinalCost() const { return last_final_cost_; }
    inline int lastSolverResult() const { return last_solver_result_; }

    /* main planning API */
    bool OptimizeTrajectory(const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
                            std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
                            std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,
                            const std::vector<Eigen::VectorXd> &pieceTimeRatios,
                            std::vector<int> singuls,double now = ros::Time::now().toSec(),double help_eps = 1.0e-4);
    MincoResult optimize(const MincoRequest &request);


    double log_sum_exp(double alpha, Eigen::VectorXd &all_dists, double &exp_sum);


  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad);

    static int progressCallback(void *func_data,
                                const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g,
                                double fx, double step, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    void VirtualTGradCost(const double &RT, const double & VT, const double & gdRT, double & gdVT, double & costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost, int trajid);
    //std::vector<Eigen::VectorXd> 
    void addPVAGradCost2CT(std::vector<Eigen::VectorXd>  &gdTs, Eigen::VectorXd &costs, const int trajid, const double trajtime);

    bool surroundGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector2d &p,
                        const Eigen::Vector2d &v,
                        Eigen::Vector2d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    double dynamicObsGradCostP( const double &omg_j,
                                const int &time_int_pena,
                                const double &t,               // current absolute time
                                const Eigen::Matrix<double, 6, 1> &beta0,
                                const Eigen::Matrix<double, 6, 1> &beta1,
                                const int& pieceid,
                                const int& K,
                                const Eigen::Vector2d &sigma, // the rear model
                                const Eigen::Vector2d &dsigma,
                                const Eigen::Vector2d &ddsigma,
                                const Eigen::Matrix2d &ego_R,
                                const Eigen::Matrix2d &help_R,
                                std::vector<Eigen::VectorXd>  &gdTs,
                                const int &trajid, const double& trajtime);
    double debugGradCheck(const int i_dp, // index of constraint point
                                         double t, // current absolute time
                                        Eigen::Vector2d sigma, // the rear model 
                                        Eigen::Vector2d dsigma,
                                        Eigen::Vector2d ddsigma,                                                                             
                                        const int trajid, const int sur_id, double res_t,Eigen::Matrix<double, 6, 2> c
                                        ,int i ,int j,double omg,double step,double wei_surround_,int K);
    bool dynamicObsCosCheck(double t_now, const Eigen::MatrixXd iniStates,  int trajid, int sur_id);

    inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                          std::vector<Eigen::MatrixXd> &vPs) const
    {
        const int M = hPs.size() - 1;

        vPs.clear();
        vPs.reserve(2 * M + 1);

        int nv;
        Eigen::MatrixXd curIH, curIV, curIOB;
        for (int i = 0; i < M; i++)
        {
            if (!geoutils::enumerateVs(hPs[i], curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);

            curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
            curIH << hPs[i], hPs[i + 1];
            if (!geoutils::enumerateVs(curIH, curIV))
            {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            vPs.push_back(curIOB);
        }

        if (!geoutils::enumerateVs(hPs.back(), curIV))
        {
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        vPs.push_back(curIOB);

        return true;
    }

    void getBoundPts(Eigen::Vector2d &position, double angle, std::vector<Eigen::Vector2d> &BoundVertices);
    void positiveSmoothedL1(const double &x, double &f, double &df);
    void positiveSmoothedL3(const double &x, double &f, double &df);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;

  };

} // namespace plan_manage
#endif
