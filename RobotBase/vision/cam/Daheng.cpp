//
// Created by jsz on 2/15/20.
//

#include "Daheng.h"

Daheng::Daheng() {
    this->status = GX_STATUS_SUCCESS;
    this->src.create(Size(640, 480), CV_8UC3);
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = (char*)"1";
    this->nFrameNum = 0;

}

Daheng::~Daheng() {
    this->status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
    free(stFrameData.pImgBuf);

    this->status = GXCloseDevice(hDevice);
    this->status = GXCloseLib();
}

int Daheng::init() {
    this->status = GXInitLib();
    if (this->status != GX_STATUS_SUCCESS) {
        printf("failed to initialize!!!!!!!!!!!!!!!\n");
        return 0;
    }

    this->status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if (status != GX_STATUS_SUCCESS || nDeviceNum == NULL) {
        printf("cannot find the device!!!!!!!!!!\n");
        return 0;
    }
    status = GXOpenDevice(&stOpenParam, &hDevice);
    //std::cout << status << "success" << std::endl;
    if (status == GX_STATUS_SUCCESS) {
        int64_t nPayLoadSize = 0;
        // acquire the image buffer size, and allocate the dynamic memory

        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);

        if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0) {
            // define the parameters pass into the GXGetImage()


            // depend on the buffer size of acquired image, m_nPayLoadSize, to allocate the buffer
            stFrameData.pImgBuf = malloc((size_t) nPayLoadSize);
            int64_t nWidth= 640;
            int64_t nHeight= 480;

            // define the image's width and height
            status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
            status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);


            // setting the exposure value
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 1500);

            //Send to acquisition command to camera
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
            return 1;
        }
    }
    return 0;

}

void Daheng::getImage(Mat &img) {
    GXFlushQueue(hDevice);
    GXGetImage(hDevice, &stFrameData, 100);
    //usleep(1);

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
        // successfully acquiring the image
        char *m_rgb_image = nullptr;

        // convert row data to Mat type
        m_rgb_image = new char[stFrameData.nWidth * stFrameData.nHeight * 3];
        src.create(stFrameData.nHeight, stFrameData.nWidth, CV_8UC3);
        DxRaw8toRGB24(stFrameData.pImgBuf,
                      m_rgb_image, stFrameData.nWidth, stFrameData.nHeight, RAW2RGB_NEIGHBOUR3,
                      DX_PIXEL_COLOR_FILTER(BAYERBG), false);
        memcpy(src.data, m_rgb_image, stFrameData.nWidth * stFrameData.nHeight * 3);

        src.copyTo(img);
//        img = src;

        nFrameNum++;

        //对图像进行处理...
        delete[]m_rgb_image;
    }
}

uint64_t Daheng::getFrameNumber() {
    return nFrameNum;
}