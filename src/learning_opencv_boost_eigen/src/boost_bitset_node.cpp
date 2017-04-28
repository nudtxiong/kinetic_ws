#include <boost/dynamic_bitset.hpp>
#include <boost/utility/binary.hpp>
#include <boost/utility.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/features2d.hpp"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/timer.hpp>
#include <boost/progress.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
using namespace boost;

using namespace cv;
void test_dynamic_bitset()
{
    // 1. 构造
    dynamic_bitset<> db1;                              // 空的dynamic_bitset
    dynamic_bitset<> db2(10);                          // 大小为10的dynamic_bitset
    dynamic_bitset<> db3('Ox16',BOOST_BINARY(10101));
    dynamic_bitset<> db4(std::string("0101"));         // 字符串构造
    // 2. resize
    db1.resize(8, true);
    assert(db1.to_ulong() == BOOST_BINARY(11111111));
    db1.resize(5);
    assert(db1.to_ulong() == BOOST_BINARY(11111));
    db1.clear();
    assert(db1.empty() && db1.size() == 0);

    // 3. push_back
    // dynamic_bitset可以像vector那样使用push_back()向容器末尾(二制数头部)追加一个值
    dynamic_bitset<> db5(5, BOOST_BINARY(01010));
    assert(db5.to_ulong() == BOOST_BINARY(01010));
    db5.push_back(true);    // 添加二进制位1
    assert(db5.to_ulong() == BOOST_BINARY(101010));
    db5.push_back(false);   // 添加二进制位0
    assert(db5.to_ulong() == BOOST_BINARY(0101010));

    // 4. block
    // dynamic_bitset使用block来存储二进制位, 一个block就可以存储64个二进制位
    assert(dynamic_bitset<>(64).num_blocks() == 1);
    assert(dynamic_bitset<>(65).num_blocks() == 2);

    // 5. 位运算
    dynamic_bitset<> db6(4, BOOST_BINARY(1010));
    db6[0] &= 1;    // 按位与运算
    db6[1] ^= 1;    // 按位异或运算
    db6[2] |= 1;    // 按位或运算
    assert(db6.to_ulong() == BOOST_BINARY_UL(1100));

    // 6. 访问元素
    dynamic_bitset<> db7(4, BOOST_BINARY(1100));
    assert(!db7.test(0) && !db7.test(1));
    assert(db7.any() && !db6.none());

    dynamic_bitset<> db9;
    db9.append(32);
    db9.append(1.8447e+19);

}
void test_read_write()
{
    int a=5;
    int i;
    dynamic_bitset<> bt(a);
    for(i=0; i<a; i++)
    {
        std::cout<<bt[i]<<" ";
    }
    std::cout<<std::endl;
    for(i=0; i<a; i++)
    {
        if(i%2==0)
        {
            bt.push_back(true);
        }else
        {
            bt.push_back(false);
        }
    }
    int len = bt.size();
    std::cout<<"bt含有："<<len<<" 个元素。";
    std::cout<<std::endl;
    //因为用的是动态分配，所以不能用sizeof去获取
    //int capa = sizeof bt;
    //cout<<"bt占用"<<capa<<"个字节。";
    int capa = bt.num_blocks() * sizeof(dynamic_bitset<>::block_type);
    std::cout<<"bt占用"<<capa<<"个字节。";
    std::cout<<std::endl;
    for(i=0; i<len; i++)
    {
       std::cout<<bt[i]<<" ";
    }
    std::cout<<std::endl;
    std::ofstream fout("/home/nubot9/nubot_ws/src/learning_opencv_boost_eigen/data/dybit.dat", std::ios_base::out | std::ios_base::binary);
    //取出原始数据，再写入文件
    int nBlockCount = bt.num_blocks();
    dynamic_bitset<>::block_type* blocks = new dynamic_bitset<>::block_type[nBlockCount];
    to_block_range(bt, blocks);
    fout.write((char*)blocks, nBlockCount * sizeof(*blocks));
    delete[] blocks;
    //关闭文件，免得打不开
    fout.close();
    std::ifstream fin("/home/nubot9/nubot_ws/src/learning_opencv_boost_eigen/data/dybit.dat",  std::ios_base::in |  std::ios_base::binary);
    dynamic_bitset<> rebt(len);
    //先读出原始数据，再恢复到对象
    dynamic_bitset<>::block_type* newBlocks = new dynamic_bitset<>::block_type[nBlockCount];
    fin.read((char*)newBlocks, nBlockCount * sizeof(*newBlocks));
    fin.close();
    from_block_range(newBlocks, newBlocks + nBlockCount, rebt);
    delete[] newBlocks;
    for(i=0; i<len; i++)
    {
        std::cout<<rebt[i]<<" ";
    }
    std::cout<<std::endl;


    dynamic_bitset<> orb_bitset(256);
    nBlockCount = orb_bitset.num_blocks();
    dynamic_bitset<>::block_type* newBlocks1 = new dynamic_bitset<>::block_type[nBlockCount];
    memset((uchar*)newBlocks1,255,nBlockCount * sizeof(*newBlocks1));
    dynamic_bitset<> rebt1(256);
    from_block_range(newBlocks1, newBlocks1 + nBlockCount, rebt1);
    int kkk=10;


    std::string m_imagedir ="/home/nubot9/nubot_ws/src/loop_detector/resources/images/SVS_L_1235603737.305831.png";
    Ptr<FeatureDetector> fd = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    Ptr<DescriptorExtractor> de = fd;
    cv::Mat image = cv::imread(m_imagedir);
    Mat mask(image.size(), CV_8UC1, Scalar(0));
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    fd->detect(image, keypoints, mask);
    de->compute(image, keypoints, descriptors);


    std::vector<__uint64_t> data2;
    data2.push_back(184392302420);
    dynamic_bitset<> rebt2(256);
    nBlockCount = rebt2.num_blocks();
    dynamic_bitset<>::block_type* newBlocks2 = new dynamic_bitset<>::block_type[nBlockCount];
    memcpy((uchar*)newBlocks2, descriptors.data, nBlockCount*sizeof(*newBlocks2));
    from_block_range(newBlocks2, newBlocks2 + nBlockCount, rebt2);
    int kkk1= sizeof(*newBlocks2);


    std::vector<__uint64_t> data_tmp(4);
    __uint64_t* data = reinterpret_cast<__uint64_t*>(descriptors.data);
    std::copy(data, data+4, data_tmp.begin());


}
void print(const boost::system::error_code& /*e*/)
{
  std::cout << "Hello, world!\n";
}
void print1(const boost::system::error_code& /*e*/,
            boost::asio::deadline_timer* t, int* count) //1s计算一次
{
    if (*count < 29)
    {
        std::cout << *count << "\n";
        ++(*count);
        t->expires_at(t->expires_at() + boost::posix_time::milliseconds(33));
        t->async_wait(boost::bind(print1,
                                  boost::asio::placeholders::error, t, count));
    }
}
void test_asio()
{
    double t2 = (double)getTickCount();
    boost::asio::io_service io;
    //boost::asio::deadline_timer t(io, boost::posix_time::seconds(5));
    int count = 0;
    boost::asio::deadline_timer t(io, boost::posix_time::milliseconds(33));
    t.async_wait(boost::bind(print1,
    boost::asio::placeholders::error, &t, &count));
   // t.async_wait(&print);
   // t.wait(); //wait 5s
    io.run();
    t2=(double)getTickCount()-t2;
    printf("in %gms\n", t2*1000/getTickFrequency());
   // std::cout << "Hello, world!\n";
}

int main()
{
    test_asio();
    //test_read_write();
    return 0;
}
