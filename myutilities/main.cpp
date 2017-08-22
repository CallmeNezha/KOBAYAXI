/// bgfx tools import
#include "bgfx.h"

/// opencv import
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

/// HMD device import
//#include "../DataDevice/hmddevice.h"
///Data device import
#include "datadevice.h"
#include <QFileDialog>
#include <QComboBox>
#include <QDialog>
#include <QPushButton>
#include <QtWidgets>
#include <Windows.h>
#include <memory>
#include "hmdTracker.hpp"

std::string getBinDir()
{
    HMODULE hModule = GetModuleHandle(NULL);
    char runnerFilePath_c[MAX_PATH];
    GetModuleFileName(hModule, runnerFilePath_c, MAX_PATH);
    std::string runnerFilePath = runnerFilePath_c;
    std::string binDir = "";
    const size_t last_slash_idx = runnerFilePath.rfind('\\');
    if (std::string::npos != last_slash_idx)
    {
        binDir = runnerFilePath.substr(0, last_slash_idx);
    }
    return binDir;
}



/// Callback
void cmdCenterAtCamera(const void* _userData);
void cmdIncImgSize    (const void* _userData);
void cmdDecImgSize    (const void* _userData);
void cmdIncHMDZ       (const void* _userData);
void cmdDecHMDZ       (const void* _userData);
void cmdIncHMDY(const void* _userData);
void cmdDecHMDY(const void* _userData);
void cmdIncHMDSize(const void* _userData);
void cmdDecHMDSize(const void* _userData);


/// Always column major
void FlipCoord(const float* _src, float* _dst)
{
    float dst[16] = { 0 };
    dst[0] = _src[0];   dst[1] = _src[1];    dst[2] = -_src[2];
    dst[4] = _src[4];   dst[5] = _src[5];    dst[6] = -_src[6];
    dst[8] = _src[8];   dst[9] = _src[9];    dst[10] = -_src[10];
    dst[12] = _src[12]; dst[13] = _src[13];  dst[14] = -_src[14];
    dst[15] = 1.f;
    bx::memCopy(_dst, dst, 16 * sizeof(float));
}


class DebugDrawApp : public entry::AppI
{
    void init(int _argc, char** _argv) BX_OVERRIDE
    {
        QApplication qApplication(_argc, _argv);
        Args args(_argc, _argv);
       
        ///
        readCameraParam();
        m_pre_pose_num = 3;
        m_image_data.data = new uint8_t[m_calib_param.height * m_calib_param.width * 3];
        /// Initialize HMD device
        //m_hmd_dev = HyLiveVR::CreateHMDDevice();
        /// Initialize Data device
        m_data_dev = HyLiveVR::CreateDataDevice();
        if (BX_UNLIKELY(NULL == m_data_dev || false == m_data_dev->Init(m_calib_param)))
        {
            std::cout << "[!!!] HMD not initialized." << std::endl;
            QMessageBox::information(NULL, "error", "HMD not initialized!", QMessageBox::Ok);
            abort();
        }
        
        /// Binding keys
        m_bindings = (InputBinding*)BX_ALLOC(entry::getAllocator(), sizeof(InputBinding) * 10);
        m_bindings[0].set(entry::Key::KeyC, entry::Modifier::None, 1, cmdCenterAtCamera, this);
        m_bindings[1].set(entry::Key::Key1, entry::Modifier::None, 1, cmdIncImgSize,     this);
        m_bindings[2].set(entry::Key::Key2, entry::Modifier::None, 1, cmdDecImgSize,     this);
        m_bindings[3].set(entry::Key::Key3, entry::Modifier::None, 1, cmdIncHMDZ,       this);
        m_bindings[4].set(entry::Key::Key4, entry::Modifier::None, 1, cmdDecHMDZ, this);
        m_bindings[5].set(entry::Key::Key5, entry::Modifier::None, 1, cmdIncHMDY, this);
        m_bindings[6].set(entry::Key::Key6, entry::Modifier::None, 1, cmdDecHMDY, this);
        m_bindings[7].set(entry::Key::Key7, entry::Modifier::None, 1, cmdIncHMDSize, this);
        m_bindings[8].set(entry::Key::Key8, entry::Modifier::None, 1, cmdDecHMDSize,       this);
        m_bindings[9].end();
        inputAddBindings("DebugDrawApp", m_bindings);

        m_width = 1920;
        m_height = 1080;
        m_debug = BGFX_DEBUG_TEXT;
        m_reset = BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X16;

        bgfx::init(args.m_type, args.m_pciId);
        bgfx::reset(m_width, m_height, m_reset);

        // Enable m_debug text.
        bgfx::setDebug(m_debug);

        // Set view 0 clear state.
        bgfx::setViewClear(0
            , BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH
            , 0x303030ff
            , 1.0f
            , 0
        );

        m_timeOffset = bx::getHPCounter();

        cameraCreate();
        cameraSetSpeed(3.f);
        const float initialPos[3] = { 0.0f, 2.0f, -3.0f };
        cameraSetPosition(initialPos);
        cameraSetVerticalAngle(0.0f);
        ddInit();


        /// Catch camera image
        {

            if (BX_UNLIKELY(false == m_data_dev->openCamera(m_camera_id)))
            {
                std::cout << "[!!!] Camera not opened." << std::endl;
                abort();
            }
            if (BX_UNLIKELY(false == m_data_dev->GetImageData(m_image_data)))
            {
                std::cout << "[!!!] Frame not readed." << std::endl;
                abort();
            }
            m_frame = cv::Mat(m_image_data.height, m_image_data.width, CV_8UC3, m_image_data.data);
            cv::cvtColor(m_frame, m_bgra, CV_BGR2BGRA);
            //cv::resize(m_bgra, m_bgra, cv::Size(800, 800));
            m_sprite = ddCreateSprite(m_bgra.cols, m_bgra.rows, m_bgra.data);
        }

        /// Create program from shaders.
        /// Set Working Directory in VS to ${SOURCE_DIR}/resources otherwise you will crash here.

        std::string basepath = getBinDir() + "/";
        m_program   = loadProgram("vs_mesh", "fs_mesh", basepath.c_str());
        std::string meshfile = basepath + "meshes/pano_basic.bin";
        m_mesh      = meshLoad(meshfile.c_str());
        //m_mesh_size = 1.f;

        /// Multi rendering

    }

    virtual int shutdown() BX_OVERRIDE
    {
        // Cleanup.

        ddDestroy(m_sprite);
        meshUnload(m_mesh);
        bgfx::destroyProgram(m_program);



        BX_FREE(entry::getAllocator(), m_bindings);
        ddShutdown();
        cameraDestroy();

        // Shutdown bgfx.
        bgfx::shutdown();

        if (m_data_dev) { delete m_data_dev; m_data_dev = NULL; }
        if (m_image_data.data) { delete[] m_image_data.data; m_image_data.data = NULL; }
        return 0;
    }

    bool update() BX_OVERRIDE
    {
        entry::WindowState state;
        if (!entry::processWindowEvents(state, m_debug, m_reset))
        {
            if (isValid(state.m_handle))
            {
                if (0 == state.m_handle.idx)
                {
                    m_width = state.m_width;
                    m_height = state.m_height;
                    m_mouseState = state.m_mouse;
                }
            }


            int64_t now = bx::getHPCounter() - m_timeOffset;
            static int64_t last = now;
            const int64_t frameTime = now - last;
            last = now;
            const double freq = double(bx::getHPFrequency());
            const double toMs = 1000.0 / freq;
            const float deltaTime = float(frameTime / freq);
            float time = float(now / freq);

            // Use debug font to print information about this example.
            bgfx::dbgTextClear();
            bgfx::dbgTextPrintf(0, 1, 0x4f, "Camera calibration debug tool.");
            bgfx::dbgTextPrintf(0, 2, 0x6f, "Description: Debug draw.");
            bgfx::dbgTextPrintf(0, 3, 0x2f, "Press c: center view on camera.");
            bgfx::dbgTextPrintf(0, 4, 0x2f, "Press 1|2: increase or decrease image depth.");
            bgfx::dbgTextPrintf(0, 5, 0x2f, "Press 3|4: increase or decrease hmd z.");
            bgfx::dbgTextPrintf(0, 6, 0x2f, "Press 5|6: increase or decrease hmd y.");
            bgfx::dbgTextPrintf(0, 7, 0x2f, "Press 7|8: increase or decrease hmd size.");
            bgfx::dbgTextPrintf(0, 8, 0x0f, "Frame: % 7.3f[ms]", double(frameTime)*toMs);

            // Update camera.
            cameraUpdate(deltaTime, m_mouseState);

            float view[16];
            cameraGetViewMtx(view);
            float proj[16];

            if (!m_calib_param.intrinsic.empty())
            {
                float fx = m_calib_param.intrinsic.at<float>(0, 0);
                float fy = m_calib_param.intrinsic.at<float>(1, 1);
                float cx = m_calib_param.intrinsic.at<float>(0, 2);
                float cy = m_calib_param.intrinsic.at<float>(1, 2);
                float width = m_calib_param.width;
                float height = m_calib_param.height;

                float fovy = (atan(cy / fy) + atan((height - cy) / fy)) / CV_PI * 180;
                float aspect = width / height;

                /// Set view and projection matrix for view 0.
                bx::mtxProj(proj, fovy, aspect, 1e-6f, 1e6f, bgfx::getCaps()->homogeneousDepth);
                bgfx::setViewTransform(0, view, proj);
                bgfx::setViewRect(0, 0, 0, uint16_t(m_width), uint16_t(m_height));

                float zero[3] = { 0 };
                float frustum[16];
                Eigen::Matrix4f mvp = getCameraPose();
                FlipCoord(mvp.data(), mvp.data());
                Eigen::Vector3f at = (mvp * Eigen::Vector4f(0.f, 0.f, 1.f, 1.f)).block(0, 0, 1, 3);
                Eigen::Vector3f eye = mvp.block(0, 3, 3, 1);
                Eigen::Vector3f up = mvp.block(0, 1, 3, 1);
                //std::cout << at - eye << " look at" << std::endl;
                bx::mtxLookAt(view, eye.data(), at.data(), up.data());
                bx::mtxProj(proj, fovy, aspect, 0.1f, 5.0f, bgfx::getCaps()->homogeneousDepth);
                bx::mtxMul(frustum, view, proj);

                ddBegin(0);
                ddDrawAxis(0.0f, 0.0f, 0.0f);
                ddDrawFrustum(frustum);

                ddDrawGrid(Axis::Y, zero, 20, 1.0f);
                ddSetTransform(NULL);

                ///
                drawEnvironment();

                ddEnd();
            }

            bgfx::frame();

            return true;
        }

        return false;
    }
public:
    ///
    void centerViewAtCamera()
    {
        Eigen::Matrix4f pose = getCameraPose();
        FlipCoord(pose.data(), pose.data());
        Eigen::Matrix4f flipY = Eigen::Matrix4f::Identity();

        cameraSetPosition(VIEW3D::getPosition(pose).data());

        //flipY(1, 1) = -1.f;;
        //pose = pose * flipY;
        VIEW3D::drawAxis(pose.data(), 1.f);
        auto bbb = VIEW3D::mtxDecompose(pose);
        cameraSetVerticalAngle(bbb.first[0]);
        cameraSetHorizontalAngle(bbb.first[1]);
        //cameraset
        /*bbb.first;
        cv::Mat r = VIEW3D::eulerAnglesToRotationMatrix(cv::Vec3f(bbb.first[0],
            bbb.first[1],
            bbb.first[2]));
        std::cout << r << std::endl;

        Eigen::Matrix4f pose_new = Eigen::Matrix4f::Identity();
        pose_new(0, 0) = r.at<float>(0, 0);
        pose_new(0, 1) = r.at<float>(0, 1);
        pose_new(0, 2) = r.at<float>(0, 2);
        pose_new(1, 0) = r.at<float>(1, 0);
        pose_new(1, 1) = r.at<float>(1, 1);
        pose_new(1, 2) = r.at<float>(1, 2);
        pose_new(2, 0) = r.at<float>(2, 0);
        pose_new(2, 1) = r.at<float>(2, 1);
        pose_new(2, 2) = r.at<float>(2, 2);
        pose_new(3, 0) = bbb.second[0];
        pose_new(3, 1) = bbb.second[1];
        pose_new(3, 2) = bbb.second[2];

        std::cout << pose_new << std::endl;*/

    }
    void incImgSize()
    {
        m_image_depth += 0.1f;
    }
    void decImgSize()
    {
        m_image_depth = std::max(0.f, m_image_depth - 0.1f);
    }
    void incHMDZ()
    {
        m_hmd_z += 0.005f;
        std::cout << "hmd z: " << m_hmd_z << std::endl;
    }
    void decHMDZ()
    {
        m_hmd_z -= 0.005f;
        std::cout << "hmd z: " << m_hmd_z << std::endl;
    }
    void incHMDY()
    {
        m_hmd_y += 0.005f;
        std::cout << "hmd y: " << m_hmd_y << std::endl;
    }
    void decHMDY()
    {
        m_hmd_y -= 0.005f;
        std::cout << "hmd y: " << m_hmd_y << std::endl;
    }
    void incHMDSize()
    {
        m_hmd_size += 0.005f;
        std::cout << "hmd size: " << m_hmd_size << std::endl;
    }
    void decHMDSize()
    {
        m_hmd_size -= 0.005f;
        std::cout << "hmd size: " << m_hmd_size << std::endl;
    }
private:

    Eigen::Matrix4f getCameraPose()
    {
        Eigen::Matrix4f camera_pose;
        Eigen::Matrix4f extrin;

        extrin = m_extrinc;
        camera_pose = m_hmd_data.track_pose * extrin.inverse();
        return camera_pose;
    }

    void drawEnvironment()
    {
        ddPush();

            float normal[3] = { 0.0f, 0.0f, 1.0f };
            float center[3] = { 0.0f, 0.0f, m_image_depth };
            if (BX_LIKELY(m_data_dev->GetImageData(m_image_data)))
            {
                m_frame = cv::Mat(m_image_data.height, m_image_data.width, CV_8UC3, m_image_data.data);
                cv::cvtColor(m_frame, m_bgra, CV_BGR2BGRA); float ratio = (float)m_frame.rows / m_frame.cols;
                cv::resize(m_bgra, m_bgra, cv::Size(800, int(800 * ratio)));
                cv::copyMakeBorder(m_bgra, m_bgra, (abs(800 - m_bgra.rows) / 2), (abs(800 - m_bgra.rows) / 2), 0, 0, cv::BORDER_CONSTANT, 0);
                cv::flip(m_bgra, m_bgra, 1);
                ddDestroy(m_sprite);
                m_sprite = ddCreateSprite(m_bgra.cols, m_bgra.rows, cv::Mat(m_bgra.t()).data);
            }

            float fx = m_calib_param.intrinsic.at<float>(0, 0);
            float width = m_calib_param.width;
            float image_size = width * m_image_depth / fx;
            ddDrawQuad(m_sprite, normal, center, image_size);
        ddPop();

        //HyLiveVR::HMDData hmd_data;
        bool pose_check = m_data_dev->GetHMDData(m_hmd_data, m_pre_pose_num);
        Eigen::Matrix4f hmd_pose = m_hmd_data.hmd_pose;  //getHmdPose();
        Eigen::Matrix4f track_pose = m_hmd_data.track_pose;
        //Eigen::Matrix4f left_handle_pose  =         //getLeftHandlePose();
        //Eigen::Matrix4f right_handle_pose =         //getRightHandlePose();
        //Eigen::Matrix4f customized_pose   =         //getCustomizedPose();
        Eigen::Matrix4f camera_pose = getCameraPose();

        trackHMD(hmd_pose/*, left_handle_pose, right_handle_pose, customized_pose*/);

        FlipCoord(hmd_pose.data(), hmd_pose.data());
        FlipCoord(track_pose.data(), track_pose.data());
        FlipCoord(camera_pose.data(), camera_pose.data());
        //FlipCoord(left_handle_pose.data(),  left_handle_pose.data());
        //FlipCoord(right_handle_pose.data(), right_handle_pose.data());
        //FlipCoord(customized_pose.data(),   customized_pose.data());


        VIEW3D::drawHMDBox(hmd_pose.data(), 0xff0000ff, true);
        VIEW3D::drawHandle(track_pose.data(), 0xfff0c0ff, true);
        //VIEW3D::drawHandle(left_handle_pose.data(),  0xfff0c0ff, true);
        //VIEW3D::drawHandle(right_handle_pose.data(), 0xfff0c0ff, true);
        //VIEW3D::drawHandle(customized_pose.data(),   0xfff0c0ff, true);
        VIEW3D::drawCamera(camera_pose.data(), 0xffff0e00);

        VIEW3D::drawAxis(camera_pose.data(), 0.1f);
        VIEW3D::drawAxis(track_pose.data(), 0.2f);
        //VIEW3D::drawAxis(left_handle_pose.data(),    0.2f      );
        //VIEW3D::drawAxis(right_handle_pose.data(), 0.2f);
        //VIEW3D::drawAxis(customized_pose.data(),   0.2f);

        /// Draw camera image
        ddSetTransform(camera_pose.data());

        /// Draw pano_basic.mesh
        Eigen::Matrix4f transform;
        
        bx::mtxSRT(transform.data(), m_hmd_size, m_hmd_size, m_hmd_size, 0.f, bx::kPi, 0.f, 0.f, m_hmd_y, m_hmd_z);
        transform = hmd_pose * transform;

        uint64_t state = 0
            | BGFX_STATE_RGB_WRITE
            | BGFX_STATE_ALPHA_WRITE
            | BGFX_STATE_DEPTH_WRITE
            | BGFX_STATE_CULL_CCW
            | BGFX_STATE_MSAA
            | BGFX_STATE_BLEND_FUNC(BGFX_STATE_BLEND_SRC_ALPHA, BGFX_STATE_BLEND_INV_SRC_ALPHA)
            ;

        meshSubmit(m_mesh, 0, m_program, transform.data(), state);
    }

    void trackHMD(Eigen::Matrix4f& hmd_pose/*, Eigen::Matrix4f& left_handle_pose, Eigen::Matrix4f& right_handle_pose, Eigen::Matrix4f& customized_pose*/)
    {
        cv::Mat hmdPose, trackerPose;
        cv::eigen2cv(hmd_pose, hmdPose);
        cv::eigen2cv(m_hmd_data.track_pose, trackerPose);

        cv::Point2f hmdPixel;
        cv::Point3f hmdPosition;
        m_upHmdTracker->Track(m_frame, trackerPose, hmdPose, hmdPixel, hmdPosition);
        cv::Mat correctedHmdPose = hmdPose.clone();
        correctedHmdPose.at<float>(0, 3) = hmdPosition.x;
        correctedHmdPose.at<float>(1, 3) = hmdPosition.y;
        correctedHmdPose.at<float>(2, 3) = hmdPosition.z;

        cv::cv2eigen(correctedHmdPose, hmd_pose);

    }

    bool readCameraParam()
    {
        std::string file_name;
        QFileDialog fileDialog;
        fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
        fileDialog.setWindowTitle("Open Camera Calib Files");
        //QFileDialog fileDialog(Q_NULLPTR, "Open Camera Calib Files", QCoreApplication::applicationDirPath(), "*.*ml");
        fileDialog.setDirectory(QString::fromStdString(getBinDir()));
        fileDialog.setNameFilter("*.*ml");
        fileDialog.show();
        if (fileDialog.exec() == QDialog::Accepted) {
            foreach(const QString &file, fileDialog.selectedFiles()) {
                file_name = file.toStdString();
            }
        }
        //std::string file_name = QFileDialog::getOpenFileName(Q_NULLPTR, "calib file", QString::fromStdString(getBinDir()), "*.*ml").toStdString();
        //std::string file_name = getBinDir() + "/out_dv.xml";
        if (file_name.empty()) {
            QMessageBox::information(NULL, "error", "Can not open file!", QMessageBox::Ok);
            return false;
        }

        //std::cout << file_name << std::endl;

        QDialog cameraSelect;
        //cameraSelect.setGeometry(QRect(100, 100, 300, 100));
        cameraSelect.setWindowTitle("cameraSelect");
        //cameraSelect->resize(100, 50);
        QComboBox cameraLists(&cameraSelect);
        cameraLists.setGeometry(QRect(20, 20, 250, 20));
        QPushButton okB(&cameraSelect);
        okB.setText("ok");
        okB.setGeometry(QRect(130, 60, 50, 30));
        QObject::connect(&okB, SIGNAL(clicked()), &cameraSelect, SLOT(accept()));


        std::vector<std::string> cameraList;
        HyLiveVR::enumerateDevice(cameraList);
        if (cameraList.empty()) {
            QMessageBox::information(NULL, "error", "Not found the camera device!", QMessageBox::Ok);
            return false;
        }
        for (int i = 0; i < cameraList.size(); i++) {
            cameraLists.addItem(QString::fromStdString(cameraList[i]));
        }

        auto  rt = cameraSelect.exec();
        if (rt == QDialog::Rejected) {
            return false;
        }
        else if (rt == QDialog::Accepted) {
            m_camera_id = cameraLists.currentIndex();
            
            //std::cout << "accepted" << std::endl;
        }

    
        std::string     file(file_name);
        cv::FileStorage fs  (file, cv::FileStorage::READ);

        BX_CHECK(fs.isOpened())
        fs["trackId"] >> (int&)m_calib_param.trackId;
        fs["image_width"] >> m_calib_param.width;
        fs["image_height"] >> m_calib_param.height;
        fs["fps"] >> m_calib_param.fps;
        fs["camera_matrix"] >> m_calib_param.intrinsic; m_calib_param.intrinsic.convertTo(m_calib_param.intrinsic, CV_32F);
        fs["distortion_coefficients"] >> m_calib_param.distortion; m_calib_param.distortion.convertTo(m_calib_param.distortion, CV_32F);
        fs["rotation"] >> m_calib_param.rotation;    m_calib_param.rotation.convertTo(m_calib_param.rotation, CV_32F);
        fs["translation"] >> m_calib_param.translation; m_calib_param.translation.convertTo(m_calib_param.translation, CV_32F);

        //m_calib_param.tvec.at<float>(1) += 0.1;
        //m_calib_param.tvec.at<float>(2) += 0.1;

        fs.release();

        cv::Mat cam_trker_mtx; cv::Rodrigues(m_calib_param.rotation, cam_trker_mtx);
        Eigen::Matrix3f ctt_e;
        ctt_e << cam_trker_mtx.at<float>(0, 0), cam_trker_mtx.at<float>(0, 1), cam_trker_mtx.at<float>(0, 2)
            , cam_trker_mtx.at<float>(1, 0), cam_trker_mtx.at<float>(1, 1), cam_trker_mtx.at<float>(1, 2)
            , cam_trker_mtx.at<float>(2, 0), cam_trker_mtx.at<float>(2, 1), cam_trker_mtx.at<float>(2, 2);
        m_extrinc = Eigen::Matrix4f::Identity();
        m_extrinc.block(0, 0, 3, 3) = ctt_e;
        m_extrinc.block(0, 3, 3, 1) = Eigen::Vector3f(m_calib_param.translation.at<float>(0), m_calib_param.translation.at<float>(1), m_calib_param.translation.at<float>(2));

        m_track_type = HyLiveVR::SubDeviceType(m_calib_param.trackId);
        m_image_depth = 3.f;
        m_hmd_z = 0.05f;
        m_hmd_y = 0.02f;
        m_hmd_size = 1.f;
        
        cv::Mat hmdOrigin = (cv::Mat_<float>(3, 1) << 0, 0, -0.07627112);
        m_upHmdTracker.reset(new HyLiveVR::HMDTracker(m_calib_param.intrinsic, m_calib_param.distortion, cam_trker_mtx, m_calib_param.translation, hmdOrigin, 0, HyLiveVR::VR2CAMERA::eHypereal2Camera, true));


    }

    entry::MouseState m_mouseState;
    InputBinding*     m_bindings;
    SpriteHandle      m_sprite;

    int64_t m_timeOffset;

    uint32_t m_width;
    uint32_t m_height;
    uint32_t m_debug;
    uint32_t m_reset;

    // video
    //cv::VideoCapture  m_video_cap;
    cv::Mat           m_frame;
    cv::Mat           m_bgra;
    float             m_image_depth;
    float             m_hmd_z;
    float             m_hmd_y;
    float             m_hmd_size;


    /// For fun
    Mesh*                  m_mesh;
    bgfx::ProgramHandle    m_program;
    //float                  m_mesh_size;

    std::unique_ptr<HyLiveVR::HMDTracker> m_upHmdTracker;


    /// HMD device
    //HyLiveVR::HMDDevice*   m_hmd_dev;
    ///Data device
    HyLiveVR::IDataDevice *m_data_dev;
    
    int m_pre_pose_num;

    HyLiveVR::HMDData m_hmd_data;
    HyLiveVR::ImageData m_image_data;
    
    HyLiveVR::SubDeviceType m_track_type;
    
    HyLiveVR::CalibParameters m_calib_param;
    Eigen::Matrix4f m_extrinc;
    int m_camera_id;
};


ENTRY_IMPLEMENT_MAIN(DebugDrawApp);

/// Callback implements
void cmdCenterAtCamera(const void* _userData)
{
    ((DebugDrawApp*)_userData)->centerViewAtCamera();
}

void cmdIncImgSize(const void* _userData)
{
    ((DebugDrawApp*)_userData)->incImgSize();
}

void cmdDecImgSize(const void* _userData)
{
    ((DebugDrawApp*)_userData)->decImgSize();
}

void cmdIncHMDZ(const void* _userData)
{
    ((DebugDrawApp*)_userData)->incHMDZ();
}

void cmdDecHMDZ(const void* _userData)
{
    ((DebugDrawApp*)_userData)->decHMDZ();
}

void cmdIncHMDY(const void* _userData)
{
    ((DebugDrawApp*)_userData)->incHMDY();
}

void cmdDecHMDY(const void* _userData)
{
    ((DebugDrawApp*)_userData)->decHMDY();
}

void cmdIncHMDSize(const void* _userData)
{
    ((DebugDrawApp*)_userData)->incHMDSize();
}

void cmdDecHMDSize(const void* _userData)
{
    ((DebugDrawApp*)_userData)->decHMDSize();
}
