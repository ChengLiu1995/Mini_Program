#include "stdafx.h"
#include <iostream>
#include <complex>
#include <math.h>
#define pi 3.1415926
using namespace std;
/*
 *refference:https://www.cnblogs.com/wuyepeng/p/9795479.html
 */
void FFT(complex<double> *X, complex<double> *outPut,int N) {
	if (N == 1) {
		//N==1的时候Complex ( cos ( 2 * PI / n * k ), sin ( 2 * PI / n * k ) )  = Complex ( cos ( 2 * PI ), sin ( 2 * PI) )=1
		outPut[0] = X[0];
		cout <<"out:" <<outPut[0] << endl;
		return;
	}
	//cout << N << endl;
	if (N % 2 != 0) {
		cout << N << "invalid" << endl;
		std::abort();
	}
	int m = N / 2;
	complex<double> *X_even = new complex<double>[m];
	complex<double> *X_odd = new complex<double>[m];
	complex<double> *O_even = new complex<double>[m];
	complex<double> *O_odd = new complex<double>[m];
	for (int i = 0; i < m; ++i) {
		X_even[i] = X[2*i];
		X_odd[i] = X[2*i+1];
	}

	FFT(X_even, O_even, m);
	FFT(X_odd, O_odd,m);

	for (int i = 0; i < m; ++i) {
		complex<double> t(cos(2 * pi*i / N), sin( 2 * pi*i / N));
		outPut[i] = O_even[i] + t * O_odd[i];
		outPut[i + m] = O_even[i] - t * O_odd[i];
	}
	delete[] X_even;
	delete[] X_odd;
	delete[] O_even;
	delete[] O_odd;
	return;
}

int main()
{
	int N = 16;
	//python生成的数据，比较python fft(np.fft.fft(x))结果一致
	float arr[16] = { 0.20222802, 0.98152401, 0.78865015, 0.45586811, 0.87956577,
		0.21095335, 0.8967297 , 0.56638587, 0.08580567, 0.37850809,
		0.96206729, 0.67502517, 0.69811634, 0.37999457, 0.24151698,
		0.36211109 };
	complex<double> *test = new complex<double>[N]; 
	complex<double> *outPut = new complex<double>[N];
	for (int i = 0; i < N; i++) {
		test[i] = arr[i];
		cout << test[i]<< endl;
	}
	FFT(test, outPut, N);
	for (int i = 0; i < N; i++) {
		cout << outPut[i].real() << "--" << outPut[i].imag() << endl;
	}
	delete[] test;
	delete[] outPut;
	system("pause");
	return 0;
};
