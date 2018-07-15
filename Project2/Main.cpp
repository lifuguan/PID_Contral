#include <iostream>
using namespace std;

//增量式PID
class PID_incremental
{
private:
	float kp;
	float ki;
	float kd;
	float target;
	float actual;
	float e;
	float e_pre_1;
	float e_pre_2;
	float A;
	float B;
	float C;
public:
	PID_incremental(float p, float i, float d);
	float pid_control(float tar, float act);
	void pid_show();
};


//增量PID
PID_incremental::PID_incremental(float p, float i, float d) :kp(p), ki(i), kd(d), e_pre_1(0), e_pre_2(0), target(0), actual(0)
{
	A = kp + ki + kd;
	B = -2 * kd - kp;
	C = kd;
	e = target - actual;
}
float PID_incremental::pid_control(float tar, float act)
{
	float u_increment;
	target = tar;
	actual = act;
	e = target - actual;
	u_increment = A * e + B * e_pre_1 + C * e_pre_2;
	e_pre_2 = e_pre_1;
	e_pre_1 = e;
	return u_increment;
}

void PID_incremental::pid_show()
{
	using std::cout;
	using std::endl;
	cout << "The infomation of this incremental PID controller is as following:" << endl;
	cout << "     Kp=" << kp << endl;
	cout << "     Ki=" << ki << endl;
	cout << "     Kd=" << kd << endl;
	cout << " target=" << target << endl;
	cout << " actual=" << actual << endl;
	cout << "      e=" << e << endl;
	cout << "e_pre_1=" << e_pre_1 << endl;
	cout << "e_pre_2=" << e_pre_2 << endl;
}


int main()
{
	//测试增量PID
	PID_incremental pid1(0.35, 0.65, 0.005);
	float target = 2000.0;
	float actual = 0;
	float pid_increment = 0.0;
	int N = 50;
	pid1.pid_show();
	cout << "target=" << target << endl;
	for (; N>0; N--)
	{
		pid_increment = pid1.pid_control(target, actual);
		actual += pid_increment;
		cout << "N=" << 50 - N << "   actual=" << actual << endl;
	}
	pid1.pid_show();		  

	system("pause");
	return 0;
}