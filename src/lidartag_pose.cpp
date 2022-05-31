#include <lidartag/lidartag.hpp>
#include <lidartag/utils.hpp>

#include <nlopt.hpp>
#include <iostream>

using namespace std;

const char *result_to_string(nlopt_result result)
{
  switch(result)
  {
    case NLOPT_FAILURE: return "FAILURE";
    case NLOPT_INVALID_ARGS: return "INVALID_ARGS";
    case NLOPT_OUT_OF_MEMORY: return "OUT_OF_MEMORY";
    case NLOPT_ROUNDOFF_LIMITED: return "ROUNDOFF_LIMITED";
    case NLOPT_FORCED_STOP: return "FORCED_STOP";
    case NLOPT_SUCCESS: return "SUCCESS";
    case NLOPT_STOPVAL_REACHED: return "STOPVAL_REACHED";
    case NLOPT_FTOL_REACHED: return "FTOL_REACHED";
    case NLOPT_XTOL_REACHED: return "XTOL_REACHED";
    case NLOPT_MAXEVAL_REACHED: return "MAXEVAL_REACHED";
    case NLOPT_MAXTIME_REACHED: return "MAXTIME_REACHED";
    default: return NULL;
  }
}

namespace BipedLab
{
double checkCost(double point, double cons1, double cons2)
{
  if (point >= cons1 && point <= cons2)
    return 0;
  else {
    return std::min(std::abs(point - cons2), std::abs(point - cons1));
  }
}
Eigen::VectorXd d_px_euler(
  double x11, double y11, double z11, double rpy11, double rpy12, double rpy13)
{
  double t2 = (rpy12 * M_PI) / 1.8e+2;
  double t3 = (rpy13 * M_PI) / 1.8e+2;
  double t4 = std::cos(t2);
  double t5 = std::cos(t3);
  double t6 = std::sin(t2);
  double t7 = std::sin(t3);
  // double test = 0;
  Eigen::VectorXd d_px(6);
  // d_px << 0.0,0.0,0.0,0.0,0.0,0.0;
  d_px << 1.0, 0.0, 0.0, 0.0,
    (z11 * t4 * M_PI) / 1.8e+2 - (x11 * t5 * t6 * M_PI) / 1.8e+2 + (y11 * t6 * t7 * M_PI) / 1.8e+2,
    t4 * M_PI * (x11 * t7 + y11 * t5) * (-1.0 / 1.8e+2);
  return d_px;
}

Eigen::VectorXd d_py_euler(
  double x11, double y11, double z11, double rpy11, double rpy12, double rpy13)
{
  double t2 = (rpy11 * M_PI) / 1.8e+2;
  double t3 = (rpy12 * M_PI) / 1.8e+2;
  double t4 = (rpy13 * M_PI) / 1.8e+2;
  double t5 = std::cos(t2);
  double t6 = std::cos(t3);
  double t7 = std::cos(t4);
  double t8 = std::sin(t2);
  double t9 = std::sin(t3);
  double t10 = std::sin(t4);
  // double test = 0;
  Eigen::VectorXd d_py(6);
  // d_py << 0.0,0.0,0.0,0.0,0.0,0.0;
  d_py << 0.0, 1.0, 0.0,
    -x11 * ((t8 * t10 * M_PI) / 1.8e+2 - (t5 * t7 * t9 * M_PI) / 1.8e+2) -
      y11 * ((t7 * t8 * M_PI) / 1.8e+2 + (t5 * t9 * t10 * M_PI) / 1.8e+2) -
      (z11 * t5 * t6 * M_PI) / 1.8e+2,
    (t8 * M_PI * (z11 * t9 + x11 * t6 * t7 - y11 * t6 * t10)) / 1.8e+2,
    x11 * ((t5 * t7 * M_PI) / 1.8e+2 - (t8 * t9 * t10 * M_PI) / 1.8e+2) -
      y11 * ((t5 * t10 * M_PI) / 1.8e+2 + (t7 * t8 * t9 * M_PI) / 1.8e+2);
  return d_py;
}

Eigen::VectorXd d_pz_euler(
  double x11, double y11, double z11, double rpy11, double rpy12, double rpy13)
{
  double t2 = (rpy11 * M_PI) / 1.8e+2;
  double t3 = (rpy12 * M_PI) / 1.8e+2;
  double t4 = (rpy13 * M_PI) / 1.8e+2;
  double t5 = std::cos(t2);
  double t6 = std::cos(t3);
  double t7 = std::cos(t4);
  double t8 = std::sin(t2);
  double t9 = std::sin(t3);
  double t10 = std::sin(t4);
  // double test = 0;
  Eigen::VectorXd d_pz(6);
  // d_pz << 0.0,0.0,0.0,0.0,0.0,0.0;
  d_pz << 0.0, 0.0, 1.0,
    x11 * ((t5 * t10 * M_PI) / 1.8e+2 + (t7 * t8 * t9 * M_PI) / 1.8e+2) +
      y11 * ((t5 * t7 * M_PI) / 1.8e+2 - (t8 * t9 * t10 * M_PI) / 1.8e+2) -
      (z11 * t6 * t8 * M_PI) / 1.8e+2,
    t5 * M_PI * (z11 * t9 + x11 * t6 * t7 - y11 * t6 * t10) * (-1.0 / 1.8e+2),
    x11 * ((t7 * t8 * M_PI) / 1.8e+2 + (t5 * t9 * t10 * M_PI) / 1.8e+2) -
      y11 * ((t8 * t10 * M_PI) / 1.8e+2 - (t5 * t7 * t9 * M_PI) / 1.8e+2);
  return d_pz;
}

// double costfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data) {
//     // x[0-2] transaltion , x[3-5] euler
//     pcl::PointCloud<LidarPoints_t>* d = reinterpret_cast<pcl::PointCloud<LidarPoints_t>*>(func_data);
//     const double box_width = 0.02;   //0.02
//     const double tag_size = (*d)[d->size()-1].tag_size;
//     double total_cost = 0;
//     double costx =0 , costy =0, costz= 0;

//     Eigen::Quaternion<float> q =  Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX())
//                                 * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
//                                 * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());

//     Eigen::Matrix3f rotation = q.matrix();

//     Eigen::Vector3f translation(x[0], x[1], x[2]);

//     Eigen::VectorXf grad_eig = Eigen::VectorXf::Zero(6);
//     // if (debug_info_) {
//     //     ROS_INFO_STREAM("Optimizing pose.... ");
//     // }
//     for(int i =0; i< d->size()-1; ++i){
//         Eigen::Vector3f p((*d)[i].point.x, (*d)[i].point.y, (*d)[i].point.z);
//         Eigen::Vector3f q = rotation * p + translation;
//         if (!grad.empty()) {
//             // Eigen::VectorXf d_px = d_px_euler(q[0],q[1],q[2],x[3],x[4],x[5]);
//             // Eigen::VectorXf d_py = d_py_euler(q[0],q[1],q[2],x[3],x[4],x[5]);
//             // Eigen::VectorXf d_pz = d_pz_euler(q[0],q[1],q[2],x[3],x[4],x[5]);
//             Eigen::VectorXf d_px(6);
//             d_px= d_px_euler(q[0], q[1], q[2], x[3], x[4], x[5]);
//             Eigen::VectorXf d_py(6);
//             d_py= d_py_euler(q[0],q[1],q[2],x[3],x[4],x[5]);
//             Eigen::VectorXf d_pz(6);
//             d_pz= d_pz_euler(q[0],q[1],q[2],x[3],x[4],x[5]);
//             if (q[0] > box_width) {
//                 grad_eig = (grad_eig + d_px).eval();
//             } else if (q[0] < -box_width) {
//                 grad_eig = (grad_eig - d_px).eval();
//             }
//             if (q[1] > (tag_size)/2) {
//                 grad_eig = (grad_eig + d_py).eval();
//             } else if (q[1] < -(tag_size)/2) {
//                 grad_eig = (grad_eig - d_py).eval();
//             }
//             if (q[2] > (tag_size)/2) {
//                 grad_eig = (grad_eig + d_pz).eval();
//             } else if (q[2] < -(tag_size)/2) {
//                 grad_eig = (grad_eig - d_pz).eval();
//             }
//         }
//         costx += checkCost(q[0], -box_width, box_width );
//         costy += checkCost(q[1], -(tag_size)/2, (tag_size)/2);
//         costz += checkCost(q[2], -(tag_size)/2, (tag_size)/2);
//     }
//     // std::cout << "gradients : " << grad_eig[0] << "  "<<grad_eig[1] << "  "<<grad_eig[2] <<"  "<<grad_eig[3]<<"  " << grad_eig[4] << "  "<<grad_eig[5]<< std::endl;
//     // grad[0] = 0;
//     // grad[1] = 0;
//     // grad[2] = 0;
//     // grad[3] = 0;
//     // grad[4] = 0;
//     // grad[5] = 0;
//     if (!grad.empty()) {
//         grad[0] = grad_eig[0];
//         grad[1] = grad_eig[1];
//         grad[2] = grad_eig[2];
//         grad[3] = grad_eig[3];
//         grad[4] = grad_eig[4];
//         grad[5] = grad_eig[5];
//         // std::cout << "gradients : " << grad[0] << "  "<<grad[1] << "  "<<grad[2] <<"  "<<grad[3]<<"  " << grad[4] << "  "<<grad[5]<< std::endl;
//     }
//     return total_cost = costx + costy + costz;;
// }
void d_px_lie_mat(
  const Eigen::MatrixXf & X1, const Eigen::MatrixXf & Y1, const Eigen::MatrixXf & Z1,
  const double & r11, const double & r12, const double & r13, Eigen::MatrixXf & dx_mat)
{
  double t2 = std::abs(r11);
  double t3 = std::abs(r12);
  double t4 = std::abs(r13);
  double t5 = utils::getSign(r11);
  double t6 = utils::getSign(r12);
  double t7 = utils::getSign(r13);
  double t8 = std::pow(r12, 2);
  double t9 = std::pow(r13, 2);
  double t10 = std::pow(t2, 2);
  double t11 = std::pow(t3, 2);
  double t12 = std::pow(t4, 2);
  double t13 = t8 + t9;
  double t14 = t10 + t11 + t12;
  double t15 = 1 / t14;
  double t17 = std::sqrt(t14);
  double t16 = std::pow(t15, 2);
  double t18 = 1 / t17;
  double t20 = std::cos(t17);
  double t21 = std::sin(t17);
  double t19 = std::pow(t18, 3);
  double t22 = t20 - 1;
  double t23 = t18 * t21;
  double t24 = r11 * t15 * t22;
  double t25 = -t24;
  dx_mat.setZero();

  dx_mat.row(0).setOnes();

  // dx_mat.row(1) = 0;

  // dx_mat.row(2) = 0;

  dx_mat.row(3) = Y1 * (-r12 * t15 * t22 - r13 * t2 * t5 * t15 * t20 + r13 * t2 * t5 * t19 * t21 +
                        r11 * r12 * t2 * t5 * t16 * t22 * 2 + r11 * r12 * t2 * t5 * t19 * t21) +
                  Z1 * (-r13 * t15 * t22 + r12 * t2 * t5 * t15 * t20 - r12 * t2 * t5 * t19 * t21 +
                        r11 * r13 * t2 * t5 * t16 * t22 * 2 + r11 * r13 * t2 * t5 * t19 * t21) -
                  X1 * (t2 * t5 * t13 * t16 * t22 * 2 + t2 * t5 * t13 * t19 * t21);

  dx_mat.row(4) =
    -X1 * (r12 * t15 * t22 * (-2) + t3 * t6 * t13 * t16 * t22 * 2 + t3 * t6 * t13 * t19 * t21) +
    Y1 * (t25 - r13 * t3 * t6 * t15 * t20 + r13 * t3 * t6 * t19 * t21 +
          r11 * r12 * t3 * t6 * t16 * t22 * 2 + r11 * r12 * t3 * t6 * t19 * t21) +
    Z1 * (t23 + r12 * t3 * t6 * t15 * t20 - r12 * t3 * t6 * t19 * t21 +
          r11 * r13 * t3 * t6 * t16 * t22 * 2 + r11 * r13 * t3 * t6 * t19 * t21);

  dx_mat.row(5) =
    -X1 * (r13 * t15 * t22 * (-2) + t4 * t7 * t13 * t16 * t22 * 2 + t4 * t7 * t13 * t19 * t21) +
    Y1 * (-t23 - r13 * t4 * t7 * t15 * t20 + r13 * t4 * t7 * t19 * t21 +
          r11 * r12 * t4 * t7 * t16 * t22 * 2 + r11 * r12 * t4 * t7 * t19 * t21) +
    Z1 * (t25 + r12 * t4 * t7 * t15 * t20 - r12 * t4 * t7 * t19 * t21 +
          r11 * r13 * t4 * t7 * t16 * t22 * 2 + r11 * r13 * t4 * t7 * t19 * t21);
}

void d_py_lie_mat(
  const Eigen::MatrixXf & X1, const Eigen::MatrixXf & Y1, const Eigen::MatrixXf & Z1,
  const double & r11, const double & r12, const double & r13, Eigen::MatrixXf & dy_mat)
{
  double t2 = std::abs(r11);
  double t3 = std::abs(r12);
  double t4 = std::abs(r13);
  double t5 = utils::getSign(r11);
  double t6 = utils::getSign(r12);
  double t7 = utils::getSign(r13);
  double t8 = std::pow(r11, 2);
  double t9 = std::pow(r13, 2);
  double t10 = std::pow(t2, 2);
  double t11 = std::pow(t3, 2);
  double t12 = std::pow(t4, 2);
  double t13 = t8 + t9;
  double t14 = t10 + t11 + t12;
  double t15 = 1 / t14;
  double t17 = std::sqrt(t14);
  double t16 = std::pow(t15, 2);
  double t18 = 1 / t17;
  double t20 = std::cos(t17);
  double t21 = std::sin(t17);
  double t19 = std::pow(t18, 3);
  double t22 = t20 - 1;
  double t23 = t18 * t21;
  double t24 = r12 * t15 * t22;
  double t25 = -t24;

  dy_mat.setZero();

  // dy_mat.row(0) = 0;

  dy_mat.row(1).setOnes();

  // dy_mat.row(2) = 0;

  dy_mat.row(3) =
    -Y1 * (r11 * t15 * t22 * (-2) + t2 * t5 * t13 * t16 * t22 * 2 + t2 * t5 * t13 * t19 * t21) +
    Z1 * (-t23 - r11 * t2 * t5 * t15 * t20 + r11 * t2 * t5 * t19 * t21 +
          r12 * r13 * t2 * t5 * t16 * t22 * 2 + r12 * r13 * t2 * t5 * t19 * t21) +
    X1 * (t25 + r13 * t2 * t5 * t15 * t20 - r13 * t2 * t5 * t19 * t21 +
          r11 * r12 * t2 * t5 * t16 * t22 * 2 + r11 * r12 * t2 * t5 * t19 * t21);

  dy_mat.row(4) = X1 * (-r11 * t15 * t22 + r13 * t3 * t6 * t15 * t20 - r13 * t3 * t6 * t19 * t21 +
                        r11 * r12 * t3 * t6 * t16 * t22 * 2 + r11 * r12 * t3 * t6 * t19 * t21) +
                  Z1 * (-r13 * t15 * t22 - r11 * t3 * t6 * t15 * t20 + r11 * t3 * t6 * t19 * t21 +
                        r12 * r13 * t3 * t6 * t16 * t22 * 2 + r12 * r13 * t3 * t6 * t19 * t21) -
                  Y1 * (t3 * t6 * t13 * t16 * t22 * 2 + t3 * t6 * t13 * t19 * t21);

  dy_mat.row(5) =
    -Y1 * (r13 * t15 * t22 * (-2) + t4 * t7 * t13 * t16 * t22 * 2 + t4 * t7 * t13 * t19 * t21) +
    X1 * (t23 + r13 * t4 * t7 * t15 * t20 - r13 * t4 * t7 * t19 * t21 +
          r11 * r12 * t4 * t7 * t16 * t22 * 2 + r11 * r12 * t4 * t7 * t19 * t21) +
    Z1 * (t25 - r11 * t4 * t7 * t15 * t20 + r11 * t4 * t7 * t19 * t21 +
          r12 * r13 * t4 * t7 * t16 * t22 * 2 + r12 * r13 * t4 * t7 * t19 * t21);
}

void d_pz_lie_mat(
  const Eigen::MatrixXf & X1, const Eigen::MatrixXf & Y1, const Eigen::MatrixXf & Z1,
  const double & r11, const double & r12, const double & r13, Eigen::MatrixXf & dz_mat)
{
  double t2 = std::abs(r11);
  double t3 = std::abs(r12);
  double t4 = std::abs(r13);
  double t5 = utils::getSign(r11);
  double t6 = utils::getSign(r12);
  double t7 = utils::getSign(r13);
  double t8 = std::pow(r11, 2);
  double t9 = std::pow(r12, 2);
  double t10 = std::pow(t2, 2);
  double t11 = std::pow(t3, 2);
  double t12 = std::pow(t4, 2);
  double t13 = t8 + t9;
  double t14 = t10 + t11 + t12;
  double t15 = 1 / t14;
  double t17 = std::sqrt(t14);
  double t16 = std::pow(t15, 2);
  double t18 = 1 / t17;
  double t20 = std::cos(t17);
  double t21 = std::sin(t17);
  double t19 = std::pow(t18, 3);
  double t22 = t20 - 1;
  double t23 = t18 * t21;
  double t24 = r13 * t15 * t22;
  double t25 = -t24;

  dz_mat.setZero();
  // dz_mat.row(0) = 0;
  // dz_mat.row(1) = 0;
  dz_mat.row(2).setOnes();
  dz_mat.row(3) =
    -Z1 * (r11 * t15 * t22 * (-2) + t2 * t5 * t13 * t16 * t22 * 2 + t2 * t5 * t13 * t19 * t21) +
    X1 * (t25 - r12 * t2 * t5 * t15 * t20 + r12 * t2 * t5 * t19 * t21 +
          r11 * r13 * t2 * t5 * t16 * t22 * 2 + r11 * r13 * t2 * t5 * t19 * t21) +
    Y1 * (t23 + r11 * t2 * t5 * t15 * t20 - r11 * t2 * t5 * t19 * t21 +
          r12 * r13 * t2 * t5 * t16 * t22 * 2 + r12 * r13 * t2 * t5 * t19 * t21);

  dz_mat.row(4) =
    -Z1 * (r12 * t15 * t22 * (-2) + t3 * t6 * t13 * t16 * t22 * 2 + t3 * t6 * t13 * t19 * t21) +
    X1 * (-t23 - r12 * t3 * t6 * t15 * t20 + r12 * t3 * t6 * t19 * t21 +
          r11 * r13 * t3 * t6 * t16 * t22 * 2 + r11 * r13 * t3 * t6 * t19 * t21) +
    Y1 * (t25 + r11 * t3 * t6 * t15 * t20 - r11 * t3 * t6 * t19 * t21 +
          r12 * r13 * t3 * t6 * t16 * t22 * 2 + r12 * r13 * t3 * t6 * t19 * t21);
  dz_mat.row(5) = X1 * (-r11 * t15 * t22 - r12 * t4 * t7 * t15 * t20 + r12 * t4 * t7 * t19 * t21 +
                        r11 * r13 * t4 * t7 * t16 * t22 * 2 + r11 * r13 * t4 * t7 * t19 * t21) +
                  Y1 * (-r12 * t15 * t22 + r11 * t4 * t7 * t15 * t20 - r11 * t4 * t7 * t19 * t21 +
                        r12 * r13 * t4 * t7 * t16 * t22 * 2 + r12 * r13 * t4 * t7 * t19 * t21) -
                  Z1 * (t4 * t7 * t13 * t16 * t22 * 2 + t4 * t7 * t13 * t19 * t21);
}

void d_px_euler_mat(
  const Eigen::MatrixXf & x11, const Eigen::MatrixXf & y11, const Eigen::MatrixXf & z11,
  const double & rpy11, const double & rpy12, const double & rpy13, Eigen::MatrixXf & dx_mat)
{
  // double t2 = (rpy12 * M_PI) / 180;
  // double t3 = (rpy13 * M_PI) / 180;
  double t4 = std::cos(rpy12);
  double t5 = std::cos(rpy13);
  double t6 = std::sin(rpy12);
  double t7 = std::sin(rpy13);

  dx_mat.setZero();
  dx_mat.row(0).setOnes();
  // dx_mat.row(1) = 0;
  // dx_mat.row(2) = 0;
  // dx_mat.row(3) = 0;
  // dx_mat.row(4) = (z11*t4*M_PI)/1.8e+2-(x11*t5*t6*M_PI)/1.8e+2+(y11*t6*t7*M_PI)/1.8e+2;
  dx_mat.row(4) = (z11 * t4 - x11 * t5 * t6 + y11 * t6 * t7);
  // dx_mat.row(5) = t4*M_PI*(x11*t7+y11*t5)*(-1.0/1.8e+2);
  dx_mat.row(5) = -t4 * (x11 * t7 + y11 * t5);
}

void d_py_euler_mat(
  const Eigen::MatrixXf & x11, const Eigen::MatrixXf & y11, const Eigen::MatrixXf & z11,
  const double & rpy11, const double & rpy12, const double & rpy13, Eigen::MatrixXf & dy_mat)
{
  // double t2 = (rpy11 * M_PI) / 180;
  // double t3 = (rpy12 * M_PI) / 180;
  // double t4 = (rpy13 * M_PI) / 180;
  double t5 = std::cos(rpy11);
  double t6 = std::cos(rpy12);
  double t7 = std::cos(rpy13);
  double t8 = std::sin(rpy11);
  double t9 = std::sin(rpy12);
  double t10 = std::sin(rpy13);

  dy_mat.setZero();
  // dx_mat.row(0) = 0;
  dy_mat.row(1).setOnes();
  // dy_mat.row(2) = 0;
  // dy_mat.row(3) = -x11*((t8*t10*M_PI)/1.8e+2-(t5*t7*t9*M_PI)/1.8e+2)-y11*((t7*t8*M_PI)/1.8e+2+(t5*t9*t10*M_PI)/1.8e+2)-(z11*t5*t6*M_PI)/1.8e+2,(t8*M_PI*(z11*t9+x11*t6*t7-y11*t6*t10))/1.8e+2;
  dy_mat.row(3) = (M_PI / 180) * (-x11 * (t8 * t10 - t5 * t7 * t9) -
                                  y11 * (t7 * t8 + t5 * t9 * t10) - z11 * t5 * t6);
  // dy_mat.row(4) = (t8*M_PI*(z11*t9+x11*t6*t7-y11*t6*t10))/1.8e+2;
  dy_mat.row(4) = t8 * (z11 * t9 + x11 * t6 * t7 - y11 * t6 * t10);
  // dy_mat.row(5) = x11*((t5*t7*M_PI)/1.8e+2-(t8*t9*t10*M_PI)/1.8e+2)-y11*((t5*t10*M_PI)/1.8e+2+(t7*t8*t9*M_PI)/1.8e+2);
  dy_mat.row(5) = (x11 * (t5 * t7 - t8 * t9 * t10) - y11 * (t5 * t10 + t7 * t8 * t9));
}

void d_pz_euler_mat(
  const Eigen::MatrixXf & x11, const Eigen::MatrixXf & y11, const Eigen::MatrixXf & z11,
  const double & rpy11, const double & rpy12, const double & rpy13, Eigen::MatrixXf & dz_mat)
{
  // double t2 = (rpy11 * M_PI) / 180;
  // double t3 = (rpy12 * M_PI) / 180;
  // double t4 = (rpy13 * M_PI) / 180;
  double t5 = std::cos(rpy11);
  double t6 = std::cos(rpy12);
  double t7 = std::cos(rpy13);
  double t8 = std::sin(rpy11);
  double t9 = std::sin(rpy12);
  double t10 = std::sin(rpy13);
  dz_mat.setZero();
  // dz_mat.row(0) = 0;
  // dz_mat_row(1) = 0;
  dz_mat.row(2).setOnes();
  // dz_mat.row(3) = x11*((t5*t10*M_PI)/1.8e+2+(t7*t8*t9*M_PI)/1.8e+2)+y11*((t5*t7*M_PI)/1.8e+2-(t8*t9*t10*M_PI)/1.8e+2)-(z11*t6*t8*M_PI)/1.8e+2;
  dz_mat.row(3) =
    (x11 * (t5 * t10 + t7 * t8 * t9) + y11 * (t5 * t7 - t8 * t9 * t10) - z11 * t6 * t8);
  // dz_mat.row(4) = t5*M_PI*(z11*t9+x11*t6*t7-y11*t6*t10)*(-1.0/1.8e+2);
  dz_mat.row(4) = -t5 * (z11 * t9 + x11 * t6 * t7 - y11 * t6 * t10);
  // dz_mat.row(5) = x11*((t7*t8*M_PI)/1.8e+2+(t5*t9*t10*M_PI)/1.8e+2)-y11*((t8*t10*M_PI)/1.8e+2-(t5*t7*t9*M_PI)/1.8e+2);
  dz_mat.row(5) = (x11 * (t7 * t8 + t5 * t9 * t10) - y11 * (t8 * t10 - t5 * t7 * t9));
}

double getCost(const Eigen::Vector4f & template_bound, Eigen::MatrixXf & transformed_points)
{
  Eigen::MatrixXf transfomed_cost_ary = (transformed_points.cwiseAbs()).colwise() - template_bound;
  Eigen::Vector3f cost_vec = transfomed_cost_ary.cwiseMax(0).rowwise().sum();
  double cost = cost_vec[0] + cost_vec[1] + cost_vec[2];

  return cost;
}

double evaluateCost(
  const Eigen::Matrix4f & H, const Eigen::MatrixXf & points, const Eigen::Vector4f & template_bound,
  Eigen::MatrixXf & transformed_points)
{
  transformed_points = H * points;  // 4 x n
  return getCost(template_bound, transformed_points);
}

double computeCost_lie(const std::vector<double> & x, std::vector<double> & grad, void * func_data)
{
  // ROS_INFO_STREAM("Inside ComputeCost");
  // x[0-2] transaltion , x[3-5] euler
  Eigen::MatrixXf * d = reinterpret_cast<Eigen::MatrixXf *>(func_data);
  int num_points = d->cols() - 1;  // last column is passed as boundary

  // for (int i = 0; i < num_points; ++i) {
  //     Eigen::Vector3f p((*d)(0, i), (*d)(1, i), (*d)(2, i));
  //     if (isnan(p(0)) || isnan(p(1)) || isnan(p(2))) {
  //         std::cout << "nan idx1 : " << i << std::endl;
  //     }
  // }
  const double box_width = (*d)(0, num_points);
  const double tag_size = (*d)(0, num_points) * 2;
  Eigen::Vector3d q(x[3], x[4], x[5]);
  Eigen::Matrix4f H = Eigen::Matrix4f::Identity(4, 4);
  H.topLeftCorner(3, 3) = utils::expSO3(q);
  H.topRightCorner(3, 1) << x[0], x[1], x[2];

  // Eigen::MatrixXf transfomed_points_mat =
  //     (H * (*d).topLeftCorner(4, num_points)); // 4 x n

  // Eigen::MatrixXf transfomed_cost_ary =
  //     (transfomed_points_mat.cwiseAbs()).colwise() - (*d).col(num_points);
  // Eigen::Vector4f cost_vec =
  //     transfomed_cost_ary.cwiseMax(0).rowwise().sum();
  Eigen::MatrixXf transfomed_points_mat;

  double cost =
    evaluateCost(H, (*d).topLeftCorner(4, num_points), (*d).col(num_points), transfomed_points_mat);
  // evaluateCost(H, (*d).topLeftCorner(4, num_points), (*d).col(num_points));

  // double total_cost = 0;
  // double costx =0 , costy =0, costz= 0;

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_x_ind =
    (transfomed_points_mat.row(0).array() > box_width).cast<float>() -
    (transfomed_points_mat.row(0).array() < -box_width).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_y_ind =
    (transfomed_points_mat.row(1).array() > tag_size / 2).cast<float>() -
    (transfomed_points_mat.row(1).array() < -tag_size / 2).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_z_ind =
    (transfomed_points_mat.row(2).array() > tag_size / 2).cast<float>() -
    (transfomed_points_mat.row(2).array() < -tag_size / 2).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_points_ind(1, 3 * (num_points));  // 1 x 3n
  transformed_points_ind << transformed_x_ind, transformed_y_ind, transformed_z_ind;

  Eigen::VectorXf grad_eig = Eigen::VectorXf::Zero(6);
  if (!grad.empty()) {
    Eigen::MatrixXf dx_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_px_lie_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5],
      dx_mat);  // 6 x n

    Eigen::MatrixXf dy_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_py_lie_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5], dy_mat);

    Eigen::MatrixXf dz_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_pz_lie_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5], dz_mat);

    Eigen::MatrixXf d_mat(6, 3 * (num_points));
    d_mat << dx_mat, dy_mat, dz_mat;
    // int nan_index;
    // int col_num;
    // for (int p = 0; p < 3*(num_points-1); ++p) {
    //     if (isnan(d_mat.row(3)(p))) {
    //         cout << "index " << p << " is nan" << endl;
    //         if ((p > (num_points-1)) && (p < 2*(num_points-1))) {
    //             col_num = p + 1 - num_points;
    //             cout << "col_num: " << col_num << endl;
    //         }
    //         std::cout << "coordinates: \n"
    //             << transfomed_points_mat.col(col_num) << std::endl;

    //         std::cout << "dmat: \n" << d_mat.col(p) << endl;
    //         nan_index = p;
    //         exit(0);
    //         break;
    //     }
    // }
    // if (d_mat.array().isNaN()){
    //     cout << "2: nan" << endl;
    //     exit(0);
    // }
    // cout << "dx row 3: " << dx_mat.row(3) << endl;
    // cout << "dx row 4: " << dx_mat.row(4) << endl;
    // cout << "dx row 5" << dx_mat.row(5) << endl;
    // cout << "dy row 3: " << dy_mat.row(3) << endl;
    // cout << "dy row 4: " << dy_mat.row(4) << endl;
    // cout << "dy row 5" << dy_mat.row(5) << endl;
    // cout << "dz row 3: " << dz_mat.row(3) << endl;
    // cout << "dz row 4: " << dz_mat.row(4) << endl;
    // cout << "dz row 5" << dz_mat.row(5) << endl;

    grad_eig = d_mat * transformed_points_ind.transpose();
    grad[0] = grad_eig[0];
    grad[1] = grad_eig[1];
    grad[2] = grad_eig[2];
    grad[3] = grad_eig[3];
    grad[4] = grad_eig[4];
    grad[5] = grad_eig[5];
    // std::cout << "gradients_new : " << grad[0] << "  "<<grad[1] << "  "<<grad[2]
    //     <<"  "<<grad[3]<<"  " << grad[4] << "  "<<grad[5]<< std::endl;
  }
  return cost;
}

double computeCost_euler(
  const std::vector<double> & x, std::vector<double> & grad, void * func_data)
{
  // ROS_INFO_STREAM("Inside ComputeCost");
  // x[0-2] transaltion , x[3-5] euler
  Eigen::MatrixXf * d = reinterpret_cast<Eigen::MatrixXf *>(func_data);
  int num_points = d->cols() - 1;  // last column is passed as boundary

  // for (int i = 0; i < num_points; ++i) {
  //     Eigen::Vector3f p((*d)(0, i), (*d)(1, i), (*d)(2, i));
  //     if (isnan(p(0)) || isnan(p(1)) || isnan(p(2))) {
  //         std::cout << "nan idx1 : " << i << std::endl;
  //     }
  // }
  const double box_width = (*d)(0, num_points);
  const double tag_size = (*d)(0, num_points) * 2;

  Eigen::Quaternion<float> q = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX()) *
                               Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY()) *
                               Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f H = Eigen::Matrix4f::Identity(4, 4);
  H.topLeftCorner(3, 3) = q.matrix();
  H.topRightCorner(3, 1) << x[0], x[1], x[2];

  // Eigen::MatrixXf transfomed_points_mat =
  //     (H * (*d).topLeftCorner(4, num_points)); // 4 x n

  // Eigen::MatrixXf transfomed_cost_ary =
  //     (transfomed_points_mat.cwiseAbs()).colwise() - (*d).col(num_points);
  // Eigen::Vector4f cost_vec =
  //     transfomed_cost_ary.cwiseMax(0).rowwise().sum();
  Eigen::MatrixXf transfomed_points_mat;

  double cost =
    evaluateCost(H, (*d).topLeftCorner(4, num_points), (*d).col(num_points), transfomed_points_mat);
  // evaluateCost(H, (*d).topLeftCorner(4, num_points), (*d).col(num_points));

  // double total_cost = 0;
  // double costx =0 , costy =0, costz= 0;

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_x_ind =
    (transfomed_points_mat.row(0).array() > box_width).cast<float>() -
    (transfomed_points_mat.row(0).array() < -box_width).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_y_ind =
    (transfomed_points_mat.row(1).array() > tag_size / 2).cast<float>() -
    (transfomed_points_mat.row(1).array() < -tag_size / 2).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_z_ind =
    (transfomed_points_mat.row(2).array() > tag_size / 2).cast<float>() -
    (transfomed_points_mat.row(2).array() < -tag_size / 2).cast<float>();  // 1 x n

  Eigen::Matrix<float, 1, Eigen::Dynamic> transformed_points_ind(1, 3 * (num_points));  // 1 x 3n
  transformed_points_ind << transformed_x_ind, transformed_y_ind, transformed_z_ind;

  Eigen::VectorXf grad_eig = Eigen::VectorXf::Zero(6);
  if (!grad.empty()) {
    Eigen::MatrixXf dx_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_px_euler_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5],
      dx_mat);  // 6 x n

    Eigen::MatrixXf dy_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_py_euler_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5], dy_mat);

    Eigen::MatrixXf dz_mat = Eigen::MatrixXf::Zero(6, num_points);
    d_pz_euler_mat(
      transfomed_points_mat.row(0), transfomed_points_mat.row(1), transfomed_points_mat.row(2),
      x[3], x[4], x[5], dz_mat);

    Eigen::MatrixXf d_mat(6, 3 * (num_points));
    d_mat << dx_mat, dy_mat, dz_mat;
    // int nan_index;
    // int col_num;
    // for (int p = 0; p < 3*(num_points-1); ++p) {
    //     if (isnan(d_mat.row(3)(p))) {
    //         cout << "index " << p << " is nan" << endl;
    //         if ((p > (num_points-1)) && (p < 2*(num_points-1))) {
    //             col_num = p + 1 - num_points;
    //             cout << "col_num: " << col_num << endl;
    //         }
    //         std::cout << "coordinates: \n"
    //             << transfomed_points_mat.col(col_num) << std::endl;

    //         std::cout << "dmat: \n" << d_mat.col(p) << endl;
    //         nan_index = p;
    //         exit(0);
    //         break;
    //     }
    // }
    // if (d_mat.array().isNaN()){
    //     cout << "2: nan" << endl;
    //     exit(0);
    // }
    // cout << "dx row 3: " << dx_mat.row(3) << endl;
    // cout << "dx row 4: " << dx_mat.row(4) << endl;
    // cout << "dx row 5" << dx_mat.row(5) << endl;
    // cout << "dy row 3: " << dy_mat.row(3) << endl;
    // cout << "dy row 4: " << dy_mat.row(4) << endl;
    // cout << "dy row 5" << dy_mat.row(5) << endl;
    // cout << "dz row 3: " << dz_mat.row(3) << endl;
    // cout << "dz row 4: " << dz_mat.row(4) << endl;
    // cout << "dz row 5" << dz_mat.row(5) << endl;

    grad_eig = d_mat * transformed_points_ind.transpose();
    grad[0] = grad_eig[0];
    grad[1] = grad_eig[1];
    grad[2] = grad_eig[2];
    grad[3] = grad_eig[3];
    grad[4] = grad_eig[4];
    grad[5] = grad_eig[5];
    // std::cout << "gradients_new : " << grad[0] << "  "<<grad[1] << "  "<<grad[2]
    //     <<"  "<<grad[3]<<"  " << grad[4] << "  "<<grad[5]<< std::endl;
  }
  return cost;

  // cout << "num points: " << num_points << endl;
  // std::cout << "coordinates outside of if: \n"
  //           << transfomed_points_mat.col(1076) << std::endl;
  // for (int i = 0; i < num_points-1; ++i) {
  //     Eigen::Vector4f p(transfomed_points_mat(0, i),
  //             transfomed_points_mat(1, i),
  //             transfomed_points_mat(2, i),
  //             transfomed_points_mat(3, i));
  //     if (isnan(p(0)) || isnan(p(1)) || isnan(p(2)) || isnan(p(3))) {
  //         std::cout << "nan idx2 : " << i << std::endl;
  //     }
  // }

  // for (int k = 0; k<num_points-1; ++k ) {
  //     for (int j = 0; j < 3; ++j) {
  //         if (isnan(transfomed_points_mat(j, k))) {
  //             transfomed_points_mat(j, k) = 0;
  //         }
  //     }
  // }

  // Eigen::Matrix4f test;
  // test << 1, -2, 3, -4, // 4
  //        -2,  3, -4, 5, // 8
  //        -3, -4, -5, 6, // 6
  //         4, -5, -6, 7; // 11
  // // Eigen::Vector4f ans = ;
  // Eigen::Vector4f vect(1,2,5,2);
  // Eigen::Matrix4f test_abs = test.cwiseAbs();
  // Eigen::Matrix4f result = test_abs.colwise() - vect;
  // cout << "matrix: \n" << test << endl;
  // cout << "abs matrix: \n" << test_abs << endl;
  // cout << "result: \n" << result << endl;
  // cout << "ans > 0: \n" << (result.array() > 0 ) << endl;
  // cout << "ans.cwisemax(0): \n" << result.array().cwiseMax(0) << endl;
  // cout << "ans.cwisemax(0).sum: \n" << result.array().cwiseMax(0).rowwise().sum() << endl;
  // exit(0);
  // Eigen::Matrix4d test;
  // test << 1, -2, 3, -4, // 4
  //      -2,  3, -4, 5, // 8
  //      -3, -4, -5, 6, // 6
  //      4, -5, -6, 7; // 11
  // int test_num = test.cols();
  // Eigen::Matrix<double, 1, Eigen::Dynamic> test1 =
  //     (test.row(0).array() > 1).cast<double>() - (test.row(0).array() < -1).cast<double>();
  // Eigen::Matrix<double, 1, Eigen::Dynamic> test2 =
  //     (test.row(1).array() > 1).cast<double>() - (test.row(1).array() < -1).cast<double>();
  // Eigen::Matrix<double, 1, Eigen::Dynamic> test3 =
  //     (test.row(2).array() > 1).cast<double>() - (test.row(2).array() < -1).cast<double>();
  // Eigen::Matrix<double, 1, Eigen::Dynamic> test_ind(1, 3*(test_num)); // 1 x 3n
  // test_ind << test1, test2, test3;

  // cout << "test: \n" << test << endl;
  // cout << "test.row(0) indx: " << test1 << endl;;
  // cout << "test.row(1) indx: " << test2 << endl;;
  // cout << "test.row(2) indx: " << test3 << endl;;
  // cout << "conca test_ind: " << test_ind << endl;;
  // exit(0);

  // Used for debug cost func
  // for(int i =0; i< (num_points-1); ++i){
  //     Eigen::Vector3d q(transfomed_points_mat.col(i)(0), transfomed_points_mat.col(i)(1), transfomed_points_mat.col(i)(2));
  //     costx += checkCost(q[0], -box_width, box_width );
  //     costy += checkCost(q[1], -(tag_size)/2, (tag_size)/2);
  //     costz += checkCost(q[2], -(tag_size)/2, (tag_size)/2);
  // }
  // total_cost = costx + costy + costz;

  // cout << "transformed_points_ind cols: " << transformed_points_ind.cols() << endl;
  // cout << "ind: " << transformed_points_ind << endl;

  // for (int i = 0; i < 3*(num_points-1); ++i) {
  //     if (isnan(transformed_points_ind(i))) {
  //         std::cout << "nan idx1 : " << i << std::endl;
  //     }
  // }
  // exit(0);

  // Eigen::MatrixXf test = Eigen::MatrixXf::Ones(1, 10);
  // Eigen::MatrixXf test1 = Eigen::MatrixXf::Ones(1, 10);
  // test1(0,1) = 10;
  // test1(0,4) = 5;
  // test(0,1) = 2;
  // test(0,5) = 4;
  // test(0,7) = 8;
  // // Eigen::Matrix<float, 1, Eigen::Dynamic> test_result = -(test.row(0).array() < 1).cast<float>();
  // std::cout << "test result : " << test << std::endl;
  // std::cout << "test1 result : " << test1 << std::endl;
  // std::cout << "test multily result : " << (test*test1) << std::endl;

  // // Eigen::VectorXf grad_eig = Eigen::VectorXf::Zero(6);
  //     grad_eig = Eigen::VectorXd::Zero(6);
  // // if (!grad.empty()) {
  //     for(int i = 0; i < num_points - 1; ++i){
  //         // Eigen::Vector3f p((*d)(0, i), (*d)(1, i), (*d)(2, i));
  //         // Eigen::Vector3f q_pre =
  //         //     H.topLeftCorner(3, 3) * p + H.topRightCorner(3, 1);
  //         Eigen::Vector4d q = transfomed_points_mat.col(i);
  //         // cout << "q: " << q_pre << endl;
  //         // cout << "q_mat: " << q << endl;
  //         // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  //         // std::exit(0);
  //         Eigen::VectorXd d_px(6);
  //         d_px= d_px_euler(q[0], q[1], q[2], x[3], x[4], x[5]);

  //         Eigen::VectorXd d_py(6);
  //         d_py= d_py_euler(q[0], q[1], q[2], x[3], x[4], x[5]);
  //         // if ((i+num_points-1) == nan_index) {
  //         //     std::cout << "d_py : " << d_py << std::endl;
  //         //     std::cout << "d_py_new : " << dy_mat.col(i) << std::endl;
  //         //     std::cout << "d_mat : " << d_mat.col(i+num_points - 1) << std::endl;
  //         //     std::cout << "coordinates:" << q << std::endl;
  //         //     std::cout << "tag size:" <<  tag_size << std::endl;
  //         // }
  //         Eigen::VectorXd d_pz(6);
  //         d_pz= d_pz_euler(q[0], q[1], q[2], x[3], x[4], x[5]);

  //         // std::cout << "d_pz : " << d_pz << std::endl;
  //         // std::cout << "d_pz_new : " << dz_mat.col(i) << std::endl;
  //         // std::cout << "d_mat : " << d_mat.col(i+2*(num_points - 1)) << std::endl;
  //         if (q[0] > box_width) {
  //             grad_eig = (grad_eig + d_px).eval();
  //             // std::cout << "d_px : " << d_px << std::endl;
  //             // std::cout << "d_px_new : " << dx_mat.col(i) << std::endl;
  //             // std::cout << "d_mat : " << d_mat.col(i) << std::endl;
  //         } else if (q[0] < -box_width) {
  //             grad_eig = (grad_eig - d_px).eval();
  //         }
  //         if (q[1] > (tag_size)/2) {
  //             grad_eig = (grad_eig + d_py).eval();
  //         } else if (q[1] < -(tag_size)/2) {
  //             grad_eig = (grad_eig - d_py).eval();
  //         }
  //         if (q[2] > (tag_size)/2) {
  //             grad_eig = (grad_eig + d_pz).eval();
  //         } else if (q[2] < -(tag_size)/2) {
  //             grad_eig = (grad_eig - d_pz).eval();
  //         }
  //     }
  //     grad[0] = grad_eig[0];
  //     grad[1] = grad_eig[1];
  //     grad[2] = grad_eig[2];
  //     grad[3] = grad_eig[3];
  //     grad[4] = grad_eig[4];
  //     grad[5] = grad_eig[5];
  //     std::cout << "gradients : " << grad[0] << "  "<<grad[1] << "  "<<grad[2]
  //         <<"  "<<grad[3]<<"  " << grad[4] << "  "<<grad[5]<< std::endl;
  // }
  // std::cout << "origin cost : " << total_cost << std::endl;
  // std::cout << "new cost : " << (cost_x + cost_y + cost_z) << std::endl;
  // return total_cost;
}

// double computeCostTBB(
//         const std::vector<double> &x,
//         std::vector<double> &grad,
//         void *func_data) {
//     // x[0-2] transaltion , x[3-5] euler
//     Eigen::MatrixXf* d = reinterpret_cast<Eigen::MatrixXf*>(func_data);
//     int num_points = d->cols();
//     const double box_width = 0.02;
//     const double tag_size = (*d)(0, num_points-1);   //0.02

//     Eigen::Quaternion<float> q =
//         Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX()) *
//         Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY()) *
//         Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
//     Eigen::Matrix4f H = Eigen::Matrix4f::Identity(4, 4);
//     H.topLeftCorner(3, 3) = q.matrix();
//     H.topRightCorner(3, 1) << x[0], x[1], x[2];

//     Eigen::Array4f template_bound;
//     template_bound << box_width, tag_size / 2, tag_size / 2, 0;
//     Eigen::MatrixXf transfomed_points_mat =
//         H * (*d).topLeftCorner(4, num_points-1);
//     Eigen::ArrayXf transfomed_cost_ary =
//         transfomed_points_mat.array().abs() - template_bound;
//     Eigen::Vector4f cost_vec =
//         transfomed_cost_ary.cwiseMax(0).rowwise().sum();
//     double cost_x = cost_vec[0];
//     double cost_y = cost_vec[1];
//     double cost_z = cost_vec[2];

//     if (!grad.empty()) {
//         tbb::concurrent_vector<float> grad_tbb(6);
//         tbb::parallel_for(int(0), num_points - 1, [&](int i){
//         //for(int i = 0; i < num_points - 1; ++i){
//             Eigen::VectorXf grad_eig = Eigen::VectorXf::Zero(6);
//             Eigen::Vector4f q = transfomed_points_mat.col(i);
//             Eigen::VectorXf d_px(6);
//             d_px= d_px_euler(q[0], q[1], q[2], x[3], x[4], x[5]);
//             Eigen::VectorXf d_py(6);
//             d_py= d_py_euler(q[0], q[1], q[2], x[3], x[4], x[5]);
//             Eigen::VectorXf d_pz(6);
//             d_pz= d_pz_euler(q[0], q[1], q[2], x[3], x[4], x[5]);
//             if (q[0] > box_width) {
//                 grad_eig = (grad_eig + d_px).eval();
//             } else if (q[0] < -box_width) {
//                 grad_eig = (grad_eig - d_px).eval();
//             }
//             if (q[1] > (tag_size)/2) {
//                 grad_eig = (grad_eig + d_py).eval();
//             } else if (q[1] < -(tag_size)/2) {
//                 grad_eig = (grad_eig - d_py).eval();
//             }
//             if (q[2] > (tag_size)/2) {
//                 grad_eig = (grad_eig + d_pz).eval();
//             } else if (q[2] < -(tag_size)/2) {
//                 grad_eig = (grad_eig - d_pz).eval();
//             }

//             for (int j = 0; j < 6; ++j) {
//                grad_tbb[j] = grad_tbb[j] + grad_eig[j];
//             }
//         //}
//         }, tbb::auto_partitioner());
//         grad[0] = grad_tbb[0];
//         grad[1] = grad_tbb[1];
//         grad[2] = grad_tbb[2];
//         grad[3] = grad_tbb[3];
//         grad[4] = grad_tbb[4];
//         grad[5] = grad_tbb[5];
//         std::cout << "gradients : " << grad[0] << "  "<<grad[1] << "  "<<grad[2]
//             <<"  "<<grad[3]<<"  " << grad[4] << "  "<<grad[5]<< std::endl;
//     }

//     return cost_x + cost_y + cost_z;;
// }

// bool LidarTag::optimizePoseGrad(ClusterFamily_t & cluster){
//     // Analytical gradient methods
//     // LD_SLSQP  LD_MMA
//     nlopt::opt opt(nlopt::LD_MMA, 6);

//     //initial guess
//     std::vector<double> x(6);
//     x[0] = cluster.initial_pose.translation[0];
//     x[1] = cluster.initial_pose.translation[1];
//     x[2] = cluster.initial_pose.translation[2];
//     x[3] = cluster.initial_pose.roll;
//     x[4] = cluster.initial_pose.pitch;
//     x[5] = cluster.initial_pose.yaw;

//     //x, y, z, r, p, y
//     std::vector<double> lb(6), ub(6);
//     lb[0] = x[0] - 15; //in meters
//     lb[1] = x[1] - 8;
//     lb[2] = x[2] - 2;
//     lb[3] = x[3] - 1.57;//in rad (180 deg)
//     lb[4] = x[4] - 1.57;
//     lb[5] = x[5] - 1.57;

//     ub[0] = x[0] + 15;
//     ub[1] = x[1] + 8;
//     ub[2] = x[2] + 2;
//     ub[3] = x[3] + 1.57;
//     ub[4] = x[4] + 1.57;
//     ub[5] = x[5] + 1.57;

//     // last column saved as other information such as tagsizes
//     int num_points = cluster.merged_data.size();
//     Eigen::MatrixXf data =
//         Eigen::MatrixXf::Ones(4, num_points + 1);
//     data.block(0, 0, 3, num_points) = cluster.merged_data.topRows(3);
//     data(0, num_points) = cluster.tag_size;

//     // pcl::PointCloud<LidarPoints_t> data = cluster.data;
//     // pcl::PointCloud<LidarPoints_t> edge_points = cluster.edge_points;
//     // data.reserve(data.size() + edge_points.size());
//     // data.insert(data.end(), edge_points.begin(), edge_points.end());
//     // edge_points.clear();
//     // LidarPoints_t p;
//     // p.tag_size = cluster.tag_size;
//     // data.push_back(p);
//     opt.set_lower_bounds(lb);
//     opt.set_upper_bounds(ub);
//     // opt.set_min_objective(costfunc_pre, &data);
//     // opt.set_min_objective(costfunc, &data);
//     opt.set_min_objective(computeCost, &data);
//     opt.set_xtol_rel(1e-5);
//     double minf;

//     try{
//         nlopt::result result = opt.optimize(x, minf);
//         Eigen::Quaternion<float> q =
//             Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX()) *
//             Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY()) *
//             Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
//         if (debug_info_) {
//             std::cout << "optimzed euler angle : "
//                       << x[3]*180/M_PI << "  "
//                       << x[4]*180/M_PI << "  "
//                       << x[5]*180/M_PI << std::endl;
//         }
//
//         Eigen::Matrix3f rotation = q.matrix();

//         Eigen::Vector3f translation(x[0], x[1], x[2]);
//         Eigen::Matrix4f homogeneous;
//         homogeneous.topLeftCorner(3,3) = rotation;
//         homogeneous.topRightCorner(3,1) = translation;
//         homogeneous.row(3) << 0,0,0,1;

//         cluster.pose_tag_to_lidar.homogeneous = homogeneous;
//         cluster.pose_tag_to_lidar.translation = translation;
//         cluster.pose_tag_to_lidar.rotation = rotation;
//         // std::cout << "found minimum at f(" << x[0] << "," << x[1] << "," << x[2] << ") = "
//         //     << std::setprecision(10) << minf << std::endl;
//         if (debug_info_) {
//             std::cout << "found minimum at \n" << homogeneous << "\n the cost is: "
//                 << std::setprecision(10) << minf << std::endl;
//         }

//     }
//     catch(std::exception &e) {
//         if (debug_info_) {
//             std::cout << "nlopt failed: " << e.what() << std::endl;
//         }
//     }
//     if (minf > optimization_percent_ * cluster.inliers / 100) {
//         return false;
//     } else {
//         return true;
//     }
// }

// return
//  3 if optimization failed, use initial pose
//  2 if optimization diverged, use initial pose
//  1 if successful
// -1 if rejected by initial guess
// -2 if rejected by coverage of area
// -3 initial guess is bad and optimization diverged
// -4 initial guess is bad and optimization failed
int LidarTag::optimizePose(ClusterFamily_t & cluster)
{
  int num_points = cluster.merged_data.cols();
  // box_width, tag size
  Eigen::Vector4f template_bound(0.02, cluster.tag_size / 2, cluster.tag_size / 2, 0);  // 4 x 1

  Eigen::MatrixXf transformed_points;  // 4 x n
  double initial_cost = evaluateCost(
    cluster.initial_pose.homogeneous, cluster.merged_data_h, template_bound, transformed_points);
  int status = 1;

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "==== optimizePose ====");
    float distance =
      std::sqrt(pow(cluster.average.x, 2) + pow(cluster.average.y, 2) + pow(cluster.average.z, 2));
    RCLCPP_DEBUG_STREAM(get_logger(), "Distance : " << distance);
    RCLCPP_DEBUG_STREAM(get_logger(), "Actual Points: " << cluster.data.size() + cluster.edge_points.size());
    RCLCPP_DEBUG_STREAM(get_logger(), "Inital Cost: " << initial_cost);
    RCLCPP_DEBUG_STREAM(get_logger(), "Cost Threshold: " << optimization_percent_ * cluster.inliers / 100);
  }

  if (initial_cost > optimization_percent_ * cluster.inliers / 100) {
    status = -1;
    if(debug_info_) {
      RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
    }

    return status;
  }

  // compute convex hull and its area
  Eigen::MatrixXf convex_hull;
  utils::constructConvexHull(transformed_points, convex_hull);
  float initial_area = utils::computePolygonArea(convex_hull);
  float coverage_area = initial_area / (cluster.tag_size * cluster.tag_size);
  // float coa_tunable = 0.75;
  if (coverage_area < coa_tunable_) {
    status = -2;
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << false);

    return status;
  }

  //initial guess
  std::vector<double> x(6);
  x[0] = cluster.initial_pose.translation[0];
  x[1] = cluster.initial_pose.translation[1];
  x[2] = cluster.initial_pose.translation[2];
  if (derivative_method_) {
    x[3] = cluster.initial_pose.roll;
    x[4] = cluster.initial_pose.pitch;
    x[5] = cluster.initial_pose.yaw;
  } else {
    Eigen::Vector3d lie_algebra = utils::logSO3(cluster.initial_pose.rotation);
    x[3] = lie_algebra[0];
    x[4] = lie_algebra[1];
    x[5] = lie_algebra[2];
  }

  //x, y, z, r, p, y
  std::vector<double> lb(6), ub(6);
  // double rpy_lb = 0.8;
  // double rpy_ub = 0.8;
  lb[0] = x[0] - 0.2;  //in meters
  lb[1] = x[1] - 0.2;
  lb[2] = x[2] - 0.2;
  lb[3] = x[3] - opt_lb_;  // 1.57 in rad (180 deg)
  lb[4] = x[4] - opt_lb_;
  lb[5] = x[5] - opt_lb_;

  ub[0] = x[0] + 0.2;
  ub[1] = x[1] + 0.2;
  ub[2] = x[2] + 0.2;
  ub[3] = x[3] + opt_ub_;
  ub[4] = x[4] + opt_ub_;
  ub[5] = x[5] + opt_ub_;

  // Numerical gradient methods
  // good: LN_PRAXIS   LN_NEWUOA_BOUND  LN_SBPLX
  // medium: LN_BOBYQA LN_NELDERMEAD
  // bad: LN_COBYLA
  // nlopt::opt opt(nlopt::LN_SBPLX, 6);

  // Analytical gradient methods
  // LD_SLSQP  LD_MMA
  // LD_TNEWTON_PRECOND_RESTART
  // LD_TNEWTON_PRECOND
  // LD_TNEWTON_RESTART
  // LD_TNEWTON

  nlopt::algorithm opt_method;
  switch (optimization_solver_) {
    // Below is numerical gradient-based methods
    case 1:
      opt_method = nlopt::LN_PRAXIS;
      break;
    case 2:
      opt_method = nlopt::LN_NEWUOA_BOUND;
      break;
    case 3:
      opt_method = nlopt::LN_SBPLX;  // recommended
      break;
    case 4:
      opt_method = nlopt::LN_BOBYQA;
      break;
    case 5:
      opt_method = nlopt::LN_NELDERMEAD;
      break;
    case 6:
      opt_method = nlopt::LN_COBYLA;
      break;
    // Below is analytical gradient-based methods
    case 7:
      opt_method = nlopt::LD_SLSQP;  // recommended   200Hz
      break;
    case 8:
      opt_method = nlopt::LD_MMA;  // recommended   120Hz
      break;
    case 9:
      opt_method = nlopt::LD_TNEWTON_PRECOND_RESTART;  // fail 90%
      break;
    case 10:
      opt_method = nlopt::LD_TNEWTON_PRECOND;  // fail 90%
      break;
    case 11:
      opt_method = nlopt::LD_TNEWTON_RESTART;  // fail 80%
      break;
    case 12:
      opt_method = nlopt::LD_TNEWTON;  // fail 90%
      break;
    case 13:
      opt_method = nlopt::LD_LBFGS;  // fail 90%
      break;
    case 14:
      opt_method = nlopt::LD_VAR1;  // fail 90%
      break;
    case 15:
      opt_method = nlopt::LD_VAR2;  // fail 90%
      break;
    default:
      opt_method = nlopt::LD_SLSQP;  // recommended
  }

  //initial guess
  // std::vector<double> x(6);
  // x[0] = cluster.initial_pose.translation[0];
  // x[1] = cluster.initial_pose.translation[1];
  // x[2] = cluster.initial_pose.translation[2];
  // x[3] = cluster.initial_pose.roll;
  // x[4] = cluster.initial_pose.pitch;
  // x[5] = cluster.initial_pose.yaw;
  // Eigen::Quaternion<float> q =
  //     Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX()) *
  //     Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY()) *
  //     Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());

  // Eigen::Matrix3f rotation_old = q.matrix();
  // Eigen::Matrix3d rotation = rotation_from_euler(x[3],x[4],x[5]);
  // cout << "homogeneous: \n" << cluster.initial_pose.homogeneous << endl;
  // cout << "rpy_old: \n" << rotation_old << endl;
  // cout << "rpy: \n" << rotation << endl;
  // exit(0);
  //x, y, z, r, p, y
  // std::vector<double> lb(6), ub(6);
  // lb[0] = x[0] - 15; //in meters
  // lb[1] = x[1] - 8;
  // lb[2] = x[2] - 2;
  // lb[3] = x[3] - 1.57; //in rad (180 deg)
  // lb[4] = x[4] - 1.57;
  // lb[5] = x[5] - 1.57;

  // ub[0] = x[0] + 15;
  // ub[1] = x[1] + 8;
  // ub[2] = x[2] + 2;
  // ub[3] = x[3] + 1.57;
  // ub[4] = x[4] + 1.57;
  // ub[5] = x[5] + 1.57;

  // last column saved as other information such as tagsizes
  // Eigen::Vector4f info_for_cost_function;
  // info_for_cost_function << cluster.tag_size, 0, 0, 0;
  Eigen::MatrixXf data(4, num_points + 1);
  data << cluster.merged_data_h, template_bound;

  // XXX: tolerance and timeout
  float x_tol = 1e-8;
  double max_time = 1.0 / 15;  // 15 Hz
  double minf;

  nlopt::opt opt(opt_method, 6);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_xtol_rel(x_tol);
  std::vector<double> steps = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};  // tx, ty, tz, r, p, y
  opt.set_default_initial_step(steps);
  if (derivative_method_) {
    opt.set_min_objective(computeCost_euler, &data);
  } else {
    opt.set_min_objective(computeCost_lie, &data);
  }

  // opt.set_maxtime(max_time);

  // [Error Code]
  // https://github.com/stevengj/nlopt/blob/
  // 2f1fa1cc5f8a3f79834de9d155d5b6ef98aa7e50/src/api/nlopt.h#L162
  // NLOPT_FAILURE = -1,         /* generic failure code */
  // NLOPT_INVALID_ARGS = -2,
  // NLOPT_OUT_OF_MEMORY = -3,
  // NLOPT_ROUNDOFF_LIMITED = -4,
  // NLOPT_FORCED_STOP = -5,
  // NLOPT_SUCCESS = 1,          /* generic success code */
  // NLOPT_STOPVAL_REACHED = 2,
  // NLOPT_FTOL_REACHED = 3,
  // NLOPT_XTOL_REACHED = 4,
  // NLOPT_MAXEVAL_REACHED = 5,
  // NLOPT_MAXTIME_REACHED = 6,

  try {
    nlopt::result result = opt.optimize(x, minf);

    // if (result != 1) {
    //     nlopt_result results_nlopt = static_cast<nlopt_result>(result);
    //     ROS_WARN_STREAM("Optimization result: " << nlopt_result_to_string(results_nlopt));

    //     return false; // timeout
    // }
    if (minf > optimization_percent_ * cluster.inliers / 1000) {
      status = -3;
      if (debug_info_) {
        RCLCPP_WARN_STREAM(get_logger(), "Optimized Cost too large: " << std::setprecision(3) << minf);
        RCLCPP_WARN_STREAM(get_logger(), "Inital Cost: " << initial_cost);
        RCLCPP_WARN_STREAM(get_logger(), "optimization_percent_: " << optimization_percent_);
        RCLCPP_WARN_STREAM(get_logger(), "cluster.inliers: " << cluster.inliers);
      }
      if (initial_cost < 0.1 * optimization_percent_ * cluster.inliers / 1000) {
        status = 2;
        if (debug_info_) {
          RCLCPP_WARN_STREAM(get_logger(), "Use initial pose.");
          RCLCPP_WARN_STREAM(get_logger(), "optimization_percent_: " << optimization_percent_);
          RCLCPP_WARN_STREAM(get_logger(), "cluster.inliers: " << cluster.inliers);
        }

        cluster.pose_tag_to_lidar.homogeneous = cluster.initial_pose.homogeneous;
        cluster.pose_tag_to_lidar.translation =
          cluster.initial_pose.homogeneous.topRightCorner(3, 1);
        cluster.pose_tag_to_lidar.rotation = cluster.initial_pose.homogeneous.topLeftCorner(3, 3);
      }

      return status;
    }

    Eigen::Matrix3f rotation;
    if (derivative_method_) {
      Eigen::Quaternion<float> q = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX()) *
                                   Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
      rotation = q.matrix();
    } else {
      Eigen::Vector3d q(x[3], x[4], x[5]);
      rotation = utils::expSO3(q);
    }

    if (debug_info_) {
      if (derivative_method_) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Optimzed euler angle : "
          "Optimzed euler angle : " << x[3] * 180 / M_PI << ", " << x[4] * 180 / M_PI << ", "
                                    << x[5] * 180 / M_PI);
      } else {
        RCLCPP_DEBUG_STREAM(get_logger(), "Optimzed lie algebra : " << x[3] << ", " << x[4] << ", " << x[5]);
      }
    }
    Eigen::Vector3f translation(x[0], x[1], x[2]);
    Eigen::Matrix4f homogeneous;
    homogeneous.topLeftCorner(3, 3) = rotation;
    homogeneous.topRightCorner(3, 1) = translation;
    homogeneous.row(3) << 0, 0, 0, 1;

    cluster.pose_tag_to_lidar.homogeneous = homogeneous;
    cluster.pose_tag_to_lidar.translation = homogeneous.topRightCorner(3, 1);
    cluster.pose_tag_to_lidar.rotation = homogeneous.topLeftCorner(3, 3);
    // std::cout << "found minimum at f(" << x[0] << "," << x[1] << "," << x[2] << ") = "
    //     << std::setprecision(10) << minf << std::endl;

    if (debug_info_) {
      nlopt_result results_nlopt = static_cast<nlopt_result>(result);
      RCLCPP_DEBUG_STREAM(get_logger(), "Optimization result: " << std::string(result_to_string(results_nlopt)));
      RCLCPP_DEBUG_STREAM(get_logger(), "Optimized cost is: " << std::setprecision(3) << minf);
      RCLCPP_DEBUG_STREAM(get_logger(), "Found minimum at \n" << homogeneous);
    }
  } catch (std::exception & e) {
    status = -4;
    if (debug_info_)
      RCLCPP_WARN_STREAM(get_logger(), "Pose optimization failed: " << e.what());
    if (initial_cost < 0.1 * optimization_percent_ * cluster.inliers / 1000) {
      status = 3;
      cluster.pose_tag_to_lidar.homogeneous = cluster.initial_pose.homogeneous;
      cluster.pose_tag_to_lidar.translation = cluster.initial_pose.homogeneous.topRightCorner(3, 1);
      cluster.pose_tag_to_lidar.rotation = cluster.initial_pose.homogeneous.topLeftCorner(3, 3);

      if (debug_info_)
        RCLCPP_WARN_STREAM(get_logger(), "Use initial pose.");
    }
  }

  if (debug_info_) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Status: " << status);
  }

  // ROS_INFO_STREAM("Optimized Cost too large: "
  //         << std::setprecision(3) << minf);
  // ROS_INFO_STREAM("Inital Cost: " << initial_cost);
  // ROS_INFO_STREAM("optimization_percent_: " << optimization_percent_);
  // ROS_INFO_STREAM("cluster.inliers: " << cluster.inliers);
  return status;
}
}  // namespace BipedLab
