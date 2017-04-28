#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <omp.h>
using namespace cv;
//cpp_learning();
#define VectorSize 20
void cpp_learning()
{

    //得到的结论，采用指针计算复杂度小，Mat转自复杂度与行列相关，vector赋值于元素个数正相关
    std::vector<cv::Mat> a(50000,cv::Mat_<float>(1,250));
    cv::Mat_<float> d(250,50000,10);
    std::vector<cv::Mat> e;
    e.reserve(500000);
    std::vector<int> f(50000,-1);
    //e.resize(50000);
    double t = (double)getTickCount();
   // #pragma omp parallel for
    for(int i = 0 ; i < 500000; i++)
    {
       //if(!a[i].empty()) int j =0;
        if(f[i] == 0) int j =0;
    }
    //a.clear();
   // a.resize(50000);
    t=(double)getTickCount()-t;
    printf("11in %gms\n", t*1000/getTickFrequency());
    //  Initialize detection structure



    long count=0;
    char tmp[64];

    double x,y;
    long count1=0;
    char tmp2[64];
    double i,j;
    long count2=0;
    char tmp3[64];
    t = (double)getTickCount();

    #pragma omp parallel sections
    {
    #pragma omp section
        {
         for( x=0;x<=0.5;x+=0.0001)
            for( y=0;y<=1;y+=0.0001)
                {
                if(x*x+y*y<=1) count1++;
                }
        }
   #pragma omp section
       {
        for( i=0.5001;i<=1;i+=0.0001)
            for( j=0;j<=1;j+=0.0001)
                {
                if(i*i+j*j<=1) count2++;
                }
        }
    }
   // count=count1+count2;
    t=(double)getTickCount()-t;
    printf("in %gms\n", t*1000/getTickFrequency());
    // 不用加速113ms

    cv::Mat mat1(10,6,CV_8U,10);
    mat1.at<uchar>(2,4) = 8;
    std::cout<<mat1;
    mat1.release();
    std::cout<<mat1;
    mat1.release();
    std::cout<<mat1;
}


class VectorComputeComplex
{
    std::vector<Mat> * images;
    std::vector<Mat> * response;
    std::vector<int> * sizes;
    cv::Mat feas;

public:
    VectorComputeComplex(std::vector<Mat>& img, std::vector<Mat>& resp, std::vector<int> size,cv::Mat & fea) :
        images( &img ), //std::vector<Mat>& img的首地址传递给了 images的第一个数/
        response( &resp ),
        sizes( &size ),
        feas(fea) //ptr
    {
      /* for(int i = 0 ; i < 5; i++) //resp中的每一个数组都指向response[0][i]，why?
       {
           for(int k = 0; k < 10; k++)
           {
               float* pData = response[0][i].ptr<float>(k);   //第i+1行的所有元素
               for(int j = 0; j < 10; j++)
                   pData[j] = float(i); //相当于
           }
       }*/
    }
};

int main()
{
/*
    OpenCV Mat class uses shared pointer and reference counting mechanism to store data and avoid unwanted deep copies.

    Everytime you read the data from FileStorage into temp, the data is updated in the same memory location and all the references to the data of temp now point to the new data. i.e. the old data is overwritten.

    When you push the Mat into the vector, data is not copied into the element of the vector. Instead, only a reference is added to the vector and the reference counter of temp is incremented. So actually, all the elements of the vector contain same data.

    You may want to push_back a deep copy of temp into the vector like this:

    rawFaceData.push_back(temp.clone());*/


    std::vector<cv::Mat> a(VectorSize,cv::Mat_<float>(10,10));
    std::vector<cv::Mat> b(VectorSize,cv::Mat_<float>(10,10)); // 这样创建的所有Mat均指向同一的内存空间
    std::vector<cv::Mat> f;
    for(int i = 0 ;i < VectorSize ;i++)
    {
      f.push_back(cv::Mat_<float>(10,10));
    }

    // f and b initialization is different
    // b中所有元素指向同一内存空间，f中同一元素指向不同内空间
    std::cout<<b.capacity()<<std::endl;
    std::vector<int>     c(VectorSize,10);
    std::cout<<c.capacity()<<std::endl;
    cv::Mat_<float> d(10000,10000,10); //

    double t = (double)getTickCount();
   // cv::Mat Mat1 = cv::Mat_<float>(10,10);
   // cv::Mat Mat2 = cv::Mat_<float>(10,10);
    cv::Mat &  Mat1 = f[0];
    cv::Mat &  Mat2 = f[1];
  //  for(int i = 0 ; i < VectorSize; i++) //resp中的每一个数组都指向response[0][i]，why?
    {
        for(int k = 0; k < 10; k++)
        {
            float* pData =  Mat1.ptr<float>(k);   //第i+1行的所有元素 why b[0] b[1] b[2] the same 区域
            for(int j = 0; j < 10; j++)
                pData[j] = float(k); //相当于
        }
        for(int k = 0; k < 10; k++)
        {
            float* pData =  Mat2.ptr<float>(k);   //第i+1行的所有元素 why b[0] b[1] b[2] the same 区域
            for(int j = 0; j < 10; j++)
                pData[j] = float(j); //相当于
        }
    }
    cv::Mat &  Mat3 = b[0];
    cv::Mat &  Mat4 = b[1];
      for(int i = 0 ; i < VectorSize; i++) //resp中的每一个数组都指向response[0][i]，why?
      {
          for(int k = 0; k < 10; k++)
          {
              float* pData =  Mat3.ptr<float>(k);   //第i+1行的所有元素 why b[0] b[1] b[2] the same 区域
              for(int j = 0; j < 10; j++)
                  pData[j] = float(k); //相当于
          }
          for(int k = 0; k < 10; k++)
          {
              float* pData =  Mat4.ptr<float>(k);   //第i+1行的所有元素 why b[0] b[1] b[2] the same 区域
              for(int j = 0; j < 10; j++)
                  pData[j] = float(j); //相当于
          }
      }
  //  b[0] = Mat1;
  //  b[1] = Mat2;
    //VectorComputeComplex vecComplex(a, b ,c,d); //相当于指向同一片区域
    std::cout<<b[0]<<std::endl;
    std::cout<<b[1]<<std::endl;
    std::cout<<b[2]<<std::endl;
    std::cout<<b[3]<<std::endl;

    std::cout<<f[0]<<std::endl;
    std::cout<<f[1]<<std::endl;
    std::cout<<f[2]<<std::endl;
    std::cout<<f[3]<<std::endl;
    t=(double)getTickCount()-t;
    printf("in %gms\n", t*1000/getTickFrequency());

    cpp_learning();

  //std::cout<<FLT_MAX<<" "<<FLT_MIN<<" "<<std::endl;    //  Initialize detection structure
    printf("\n double MAX=%le, MIN=%le\搜索 \n", FLT_MAX, FLT_MIN);
    float mm = -FLT_MAX;
    std::cout<<mm;
    return 0;
}
