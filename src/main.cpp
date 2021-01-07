#include "sample.h"

int main(int argc, char *argv[])
{
	sample sp;
	sp.change2DExposureTime();
	sp.change3DExposureTime();
	sp.get2DImage();
	sp.getDepthImage();
	sp.getPointCloud();
    return 0;
}
