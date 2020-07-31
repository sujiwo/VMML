#include <iostream>
#include <opencv2/core.hpp>

using namespace std;

typedef cv::Mat_<float> Matf;
typedef cv::Mat_<cv::Vec3f> Matf3;


int main(int argc, char *argv[])
{
	Matf3 image(10, 10);

	float x = 0;
	for (auto it=image.begin(); it!=image.end(); ++it) {
		x+=1.0;
		(*it)[0] = x;
		(*it)[1] = x+0.5;
		(*it)[2] = x+0.75;
	}

	cout << image << endl;

	return 0;
}
