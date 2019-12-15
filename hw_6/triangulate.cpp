//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <ctime>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    const int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
    }
    
    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */

    //step1:constuct D
    Eigen::Matrix<double, 2*(10 - start_frame_id), 4> D;
    for (int i = start_frame_id; i < end_frame_id; ++i) {

        Eigen::Matrix<double,3,4> Tcw;
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw*camera_pose[i].twc;//注意是-:Pc=Pc0-tcw
        Tcw<< Rcw, tcw;

        //rows = mat.rows(2);  //  获取第3行
        Eigen::Vector2d uv = camera_pose[i].uv ;

        D.block(2*(i - start_frame_id), 0, 1,4) = uv[0]*Tcw.row(2) - Tcw.row(0);
        D.block(2*(i - start_frame_id)+1, 0, 1,4) = uv[1]*Tcw.row(2) - Tcw.row(1);


    }
    std::cout<<"D Matrix is :"<<D<<std::endl;

    //step2:recale D,找到D中最大元素
    Eigen::MatrixXd::Index maxRow, maxCol;
    double max = D.cwiseAbs().maxCoeff(&maxRow,&maxCol);
    //注意是D的绝对值中最大元素的D.cwiseAbs().maxCoeff()
    std::cout << "Max = \n" << max <<"行:"<<maxRow<<"列"<<maxCol<< std::endl;

    Eigen::MatrixXd DtD((D/max).transpose()*(D/max));

    clock_t time_stt = clock();



    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(DtD, Eigen::ComputeThinU | Eigen::ComputeThinV );

    clock_t time_stt1 = clock();


    std::cout<<"SVD分解，耗时:\n"<<(clock()-time_stt1)/(double) CLOCKS_PER_SEC<<"ms"<<std::endl; 

     // 构建SVD分解结果 Eigen::MatrixXd U = svd_holder.matrixU(); Eigen::MatrixXd V = svd_holder.matrixV(); 
     Eigen::MatrixXd S(svd_holder.singularValues()); 
     std::cout<<"singularValues :\n"<<D<<std::endl;

     //step3:判断分解是否有效,然后求解y
     if(std::abs(svd_holder.singularValues()[3]/svd_holder.singularValues()[2]) < 1e-2){

        Eigen::Vector4d U4 = (max*svd_holder.matrixU().rightCols(1));//最后一列
        P_est = (U4/U4[3]).head(3);

     }
     else{

        std::cout<<"这次求解无效"<<std::endl;
     }

 
    /* your code end */
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    return 0;
}
