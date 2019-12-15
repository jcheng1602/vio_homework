#pragma once
#include <iostream>
#include <fstream>



class Record{

private:

    std::ofstream ofs_lambda;
    std::string fn_lambda;
    std::string Flags_log_dir;


public:
    Record(){

        Flags_log_dir="/home/ubuntu/ros_catkin/src/CurveFitting_LM/build/app";

        char buffer[80];
        std::time_t now = std::time(NULL);
        std::tm* pnow = std::localtime(&now);
        std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);

        fn_lambda = Flags_log_dir + "/lambdas_" + std::string(buffer) + ".csv";
        ofs_lambda.open(fn_lambda.c_str(), std::ios::out);

   
    }

    ~Record(){}

    void add(int& iters, double& lambda)
    {
        std::cout<<"lambda has been accpecter !!"<<std::endl;
            
        ofs_lambda << iters << "\t" 
            << lambda << "\t" 
            << std::endl;
        // ROS_INFO_STREAM_ONCE("/changed_msf has published !!");
    }


    void add(int& iters, double& a, double& b, double& c)
    {

           
        std::cout<<"abc has been accpecter !!"<<std::endl;
            
        ofs_lambda << iters << "\t" 
            << a << "\t" 
            << b << "\t" 
            << c << "\t" 
            << std::endl;
    }

   
};


