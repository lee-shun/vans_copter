#ifndef CUSTOMPID_H
#define CUSTOMPID_H

//自定义 PID 类，计算各部分结果
class customPID
{
    private:
        double p_gain;
        double i_gain;
        double d_gain;
        double I_limit;
        // double in_cur;
        double result;
        double xbuf[1000];
        double ybuf[1000];
        int frame, bufLen;

    public: 
        customPID();
        // N 是 PID 求 D 时候的缓存长度
        // i_limit限幅
        customPID(int N, double i_limit);
        ~customPID();
        //获取比例部分
        double P_gain();
        //获取积分部分
        double I_gain();
        //获取微分部分
        double D_gain();
        //获取输出结果(当前帧)
        double Result();
        //获取当前帧
        int Frame();
        //根据当前帧信息更新PID输出， dt 是 前后帧的时间间隔， t_cur 是当前时间用于算前后时间间隔， in 是信号输入， p、i、d 是三个增益
        void Update(double dt, double t_cur, double in, double p, double i, double d);
};
#endif