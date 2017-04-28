#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <string>
#include <iostream>
#include <fstream>
using namespace Eigen;
void eigen_definition()
{
    typedef Matrix3f FixedXD;
    FixedXD x;
    int size =9;
    int low=1;
    int high = 9;
    int value =9;
    x = FixedXD::Zero();
    x = FixedXD::Ones();
    x = FixedXD::Constant(value);
    x = FixedXD::Random();
    x.setZero();
    x.setOnes();
    x.setConstant(value);
    x.setRandom();
    std::cout<<"x"<<x<<std::endl;

    typedef Eigen::Matrix<unsigned, 1, Eigen::Dynamic> RowXu;
    RowXu seen_points_working_vector;
    seen_points_working_vector.setLinSpaced(4, 1, 4);

    std::cout<<"RowXu "<<seen_points_working_vector<<std::endl;
}

int main()
{
   eigen_definition();
   return 0;
}
