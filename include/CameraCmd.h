#ifndef CAMERACMD_H
#define CAMERACMD_H

enum ImageType {
	DEPTH = 1,
	COLOR = 2,
	COLOR_DEPTH = COLOR | DEPTH,
};

enum NetCamCmd {
	CaptureImage = 0,
	CaptureGratingImage = 8,
	GetCameraIntri = 11,
	GetCameraStatus = 19,
	SetCameraParameter = 25,
	GetCameraParameter = 26,
};

constexpr int Encode32FBias = 32768;

#endif // CAMERACMD_H
