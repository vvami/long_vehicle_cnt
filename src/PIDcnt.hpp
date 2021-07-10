
class PIDcnt
{
public:

PIDcnt(double Kp, double Ki, double Kd, double t, double L, double H)

{
    kp = Kp;
    ki = Ki;
    kd = Kd;
    ts = t;
    Ip = 0;
    low = L;
    high = H;
    prev_err = 0;
}

double Compute(double err)
{
    double P = kp * err;
    double D = kd * ( (err-prev_err)/ts );
    prev_err = err;
    double I = Ip + ki * err * ts;
    double U = P + I + D;
    Ip = I;
    
    if(U>high)
    {
        U = high;
    }
    else if (U<low)
    {
        U = low; 
    }
    
    return U;

}


private:
double kp;
double ki;
double kd;
double prev_err;
double Ip;
double low; //low saturation limit
double high; //high saturation limit
double ts; //sampling time
};

