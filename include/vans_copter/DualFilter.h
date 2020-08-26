#ifndef DUALFILTER_H
#define DUALFILTER_H
// #include <vector>
// #include <list>
// #include <numeric>

// class dataBuf
// {
//     private:
//         int Length;
//         // std::vector<double> buf;
//         // std::list<double> buf;
//         double V1[512]={};
//     public:
//         dataBuf();
//         dataBuf(int L);
//         ~dataBuf();
//         void UpDate(double in);
//         int GetLength();
//         double GetSum();
//         double GetAver();
// };

//缓存更新前移
//a 一维数组
//N 数组长度
//in 更新值
void ArrayUpdate(double *a, int N, double in);
//缓存求和， 滤除最大最小值
//a 一维数组
//N 数组长度
double ArraySum(double *a, int N);
//求缓存均值
//a 一维数组
//N 数组长度
double ArrayAver(double *a, int N);
//向量积
//a 一维数组
//b 一维数组
//N 数组长度
double AddArray(double *a, double *b, int N);
//双级滤波，滑动平均滤波 + 高斯滤波
//buf1 一级滤波缓存数组
//buf2 二级滤波缓存数组
//sigma 二级缓存均值差
//N 数组长度
//signal 输入信号
double dualFilter(double *buf1, double *buf2, int N, double signal, double sigma);
//近似求导
//xbuf 时间缓存数组
//ybuf 信号缓存数组
//N 数组长度
//t 当前帧时间
//signal 输入信号
double DerivFun(double *xbuf, double *ybuf, int N, double t, double signal);
//得到弹道系的X分量
double X_Yaw_remap(double X, double Y, double Yaw);
//得到弹道系的Y分量
double Y_Yaw_remap(double X, double Y, double Yaw);
#endif