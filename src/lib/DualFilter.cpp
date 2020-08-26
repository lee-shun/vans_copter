#include <math.h>
#include "vans_copter/DualFilter.h"

// using namespace std;
// dataBuf::dataBuf(){}
// dataBuf::dataBuf(int L)
// {
//     this->Length=L;
//     // this->buf.assign(L,0.0);
// }
// dataBuf::~dataBuf(){}
// void dataBuf::UpDate(double in)
// {
//     // this->buf.pop_front();
//     // this->buf.push_back(in);
//     for(int i=0;i<(this->Length-1);i++)
//     {
//         this->V1[i]=this->V1[i+1];
//     }
//     this->V1[this->Length-1]=in;
// }
// int dataBuf::GetLength()
// {
//     return this->Length;
// }
// double dataBuf::GetSum()
// {
//     double sum=0;
//     for(int i=0;i<this->Length;i++)
//     {
//         sum=sum+this->V1[i];
//     }
//     return sum;
// }
// double dataBuf::GetAver()
// {
//     double Length_=this->Length;
//     double sum=0;
//     for(int i=0;i<this->Length;i++)
//     {
//         sum=sum+this->V1[i];
//     }
//     return (sum/Length_);
// }

void ArrayUpdate(double *a, int N, double in)
{
    for(int i=0;i<(N-1);i++)
    {
        a[i]=a[i+1];
    }
    a[N-1]=in;
}
double ArraySum(double *a, int N)
{
    double sum=0;
    double max=0;
    double min=0;
    double K=N;

    for(int i=0;i<N;i++)
    {
        sum=sum+a[i];
        if(a[i]>=max)
        {
            max=a[i];
        }
        if(a[i]<=min)
        {
            min=a[i];
        }
    }
    double sumtemp=sum-max-min;
    sum=sumtemp + 2.0*(sumtemp/(K-2.0));//滤出最大最小，补充失去均值
    return sum;
}
double ArrayAver(double *a, int N)
{
    double sum=ArraySum(a,N);
    double N_=N;
    return (sum/N_);
}
double AddArray(double *a, double *b, int N)
{
    double sum=0;
    for(int i=0;i<N;i++)
    {
        sum=sum+a[i]*b[i];//向量积
    }
    return sum;
}
double dualFilter(double *buf1, double *buf2, int N, double signal, double sigma)
{
    ArrayUpdate(buf1, N, signal);
    double filtTemp=ArrayAver(buf1, N);
    double GaussTemp[N]={1.0};
    for(int i=0;i<N;i++)
    {
        GaussTemp[i]=exp(-(i-(N-1))*(i-(N-1))/(2*sigma*sigma))/(sigma*sqrt(2*M_PI))*2;
    }
    ArrayUpdate(buf2, N, filtTemp);
    double result=AddArray(buf2, GaussTemp, N);
    return result;
}
double DerivFun(double *xbuf, double *ybuf, int N, double t, double signal)
{
    ArrayUpdate(xbuf, N, t);
    ArrayUpdate(ybuf, N, signal);
    double sum1=AddArray(xbuf, ybuf, N);
    double sum2=AddArray(xbuf, xbuf, N);
    double K=N;
    double x_=ArrayAver(xbuf, N);
    double y_=ArrayAver(ybuf, N);
    double result=(sum1-K*x_*y_)/(sum2-K*x_*x_);
    return result;
}
double X_Yaw_remap(double X, double Y, double Yaw)
{
    double product=X*cos(Yaw) + Y*sin(Yaw);
    return product;
}
double Y_Yaw_remap(double X, double Y, double Yaw)
{
    double product=-X*sin(Yaw) + Y*cos(Yaw);
    return product;
}