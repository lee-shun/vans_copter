#include "vans_copter/customPID.h"
#include "vans_copter/DualFilter.h"

customPID::customPID(int N, double i_limit)
{
    this->xbuf[N]={};
    this->ybuf[N]={};
    this->bufLen=N;
    this->I_limit=i_limit;
}
customPID::customPID()
{
    this->frame=0;
}
customPID::~customPID(){}

double customPID::P_gain()
{
    return this->p_gain;
}
double customPID::I_gain()
{
    return this->i_gain;
}
double customPID::D_gain()
{
    return this->d_gain;
}
double customPID::Result()
{
    return this->result;
}
int customPID::Frame()
{
    return this->frame;
}
void customPID::Update(double dt, double t_cur, double in, double p, double i, double d)
{
    if(this->frame==0)
    {
        // this->in_cur=in;
        this->i_gain=0;
    }
    this->p_gain=in;
    this->i_gain+=in*dt;
    if(this->i_gain >= this->I_limit)
        this->i_gain=this->I_limit;
    if(this->i_gain <= (-this->I_limit))
        this->i_gain=(-this->I_limit);
    this->d_gain=DerivFun(this->xbuf, this->ybuf, this->bufLen, t_cur, in);
    // this->d_gain=(in-this->in_cur)/dt;
    // this->in_cur=in;
    this->result=p*p_gain + i*i_gain + d*d_gain;
    this->frame++;
}
