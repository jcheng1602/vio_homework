#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.h"
#include "sophus/se3.h"
using namespace std; 

int main( int argc, char** argv )
{
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    
    std::cout << "R :"<< endl << R <<endl;

    Sophus::SO3 SO3_R(R);               // 从旋转矩阵构造Sophus::SO(3)
    Eigen::Quaterniond q(R);            // 从旋转矩阵构造四元数

    // 李代数更新
    Eigen::Vector3d update_so3(0.01, 0.02, 0.03); //更新量
    Sophus::SO3 SO3_updated = SO3_R * Sophus::SO3::exp(update_so3);
    std::cout<<"SO3 updated = "<< endl << SO3_updated.matrix() <<endl;


    // Sophus::SO3::expMatrix( update_so3 );
    
    //四元数更新
    Eigen::Quaterniond q_update(1, update_so3(0)/2, update_so3(1)/2, update_so3(2)/2); 
    Eigen::Quaterniond q_updated = (q * q_update).normalized(); //四元数归一化
    std::cout<<"q2R = "<< endl << q_updated.toRotationMatrix() <<endl;

    return 0;
}

