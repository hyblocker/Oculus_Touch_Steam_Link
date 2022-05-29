#include <openvr_driver.h>
// #include "json.h" // im too lazy to figure out how to use this api so ima be lazy :D
#include "driverlog.h"
#include <filesystem>

#define USE_MUTEX 1

#define CREATE_CONTROLLERS 1

#define USE_SHARE_MEM_BUFFER 1

#define DO_SKELETON 1       //enable the common skeleton code, and the right hand skeleton
#define DO_LSKELETON 1      // enable the left hand skeleton, depends on the above, but allows simultanously comparing legacy and skeleton input by disabling the left hand skeleton

#include <OVR_CAPI.h>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <iostream>

#if defined( _WINDOWS )
#include <windows.h>
#endif

using namespace vr;
#if USE_MUTEX
HANDLE comm_mutex;
#endif

struct shared_buffer {
    ovrInputState input_state;
    uint32_t vrEvent_type;
    float vib_amplitude[2];
    float vib_frequency[2];
    float vib_duration_s[2];
    bool vib_valid[2];
};

#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
    HmdQuaternion_t quat;
    quat.w = w;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    return quat;
}

inline void HmdMatrix_SetIdentity(HmdMatrix34_t* pMatrix)
{
    pMatrix->m[0][0] = 1.f;
    pMatrix->m[0][1] = 0.f;
    pMatrix->m[0][2] = 0.f;
    pMatrix->m[0][3] = 0.f;
    pMatrix->m[1][0] = 0.f;
    pMatrix->m[1][1] = 1.f;
    pMatrix->m[1][2] = 0.f;
    pMatrix->m[1][3] = 0.f;
    pMatrix->m[2][0] = 0.f;
    pMatrix->m[2][1] = 0.f;
    pMatrix->m[2][2] = 1.f;
    pMatrix->m[2][3] = 0.f;
}


// keys for use with the settings API
static const char* const k_pch_Sample_Section = "driver_sample";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";
static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------

class CWatchdogDriver_Sample : public IVRWatchdogProvider
{
public:
    CWatchdogDriver_Sample()
    {
        m_pWatchdogThread = nullptr;
    }
          
    virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
    virtual void Cleanup();

private:
    std::thread* m_pWatchdogThread;
};

CWatchdogDriver_Sample g_watchdogDriverNull;

bool g_bExiting = false;

// bool s_useOculusTrackingSpace = false;
bool s_spawnHipTracker        = true;
bool s_spawnRightLegTracker   = true;
bool s_spawnLeftLegTracker    = true;


void WatchdogThreadFunction()
{
    while (!g_bExiting)
    {
        // for the other platforms, just send one every five seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));
        vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_Controller);
    }
}

EVRInitError CWatchdogDriver_Sample::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());
    DriverLog("Started OculusTouchLink driver\n");

    // Watchdog mode on Windows starts a thread that listens for the 'Y' key on the keyboard to 
    // be pressed. A real driver should wait for a system button event or something else from the 
    // the hardware that signals that the VR system should start up.
    g_bExiting = false;
    m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
    if (!m_pWatchdogThread)
    {
        DriverLog("Unable to create watchdog thread\n");
        return VRInitError_Driver_Failed;
    }

    return VRInitError_None;
}


void CWatchdogDriver_Sample::Cleanup()
{
    g_bExiting = true;
    if (m_pWatchdogThread)
    {
        m_pWatchdogThread->join();
        delete m_pWatchdogThread;
        m_pWatchdogThread = nullptr;
    }

    CleanupDriverLog();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CSampleControllerDriver : public vr::ITrackedDeviceServerDriver                                               
{                                                                                                                                             // hand_offset({ 0.01071,0.04078,-0.04731 }), hand_offset2({-0.003,-0.101,0.0089 })
public:                                                                                                                                      //x = 0.00571 y = 0.04078 z = -0.03531 x2 =-0.000999998 y2 = -0.1 z = 0.0019
    CSampleControllerDriver(ovrSession mSession, bool isRightHand, bool isWaist, shared_buffer *comm_buffer/*, ovrVector3f overall_offset, ovrQuatf overall_rotation*/): mSession(mSession), isRightHand(isRightHand), isWaist(isWaist), comm_buffer(comm_buffer), hand_offset({ 0.00571,0.04078,-0.03531 }), hand_offset2({ -0.000999998,-0.1, 0.0019 })/*, overall_offset(overall_offset), overall_rotation(overall_rotation)*/
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
        m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
        if (isWaist){
            m_sSerialNumber = "LHR-OCULUS_VROBJECT";
            m_sModelNumber = "TouchLink Tracker (VR Object)";
        } else if (isRightHand) {
            m_sSerialNumber = "LHR-OCULUS_RIGHT";
            m_sModelNumber = "TouchLink Tracker (Right)";
        } else {
            m_sSerialNumber = "LHR-OCULUS_LEFT";
            m_sModelNumber = "TouchLink Tracker (Left)";
        }
    }

    virtual ~CSampleControllerDriver()
    {
    }

    virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
    {
        m_unObjectId = unObjectId;
        m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);
        ovrHmdDesc hmd_desc = ovr_GetHmdDesc(mSession);
        switch (hmd_desc.Type)
        {
            case ovrHmd_CV1:
                if (isWaist) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-CV1_VROBJECT");
                } else if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-CV1_RIGHT");
                } else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-CV1_LEFT");
                }
            break;
            case ovrHmd_RiftS:
                if (isWaist) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-RIFTS_VROBJECT");
                } else if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-RIFTS_RIGHT");
                } else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-RIFTS_LEFT");
                }
            case ovrHmd_Quest:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-QUEST_RIGHT");
                } else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-QUEST_LEFT");
                }
            case ovrHmd_Quest2:
                if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-QUEST2_RIGHT");
                } else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-QUEST2_LEFT");
                }
            default:
                if (isWaist) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-OCULUS_VROBJECT");
                } else if (isRightHand) {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-OCULUS_RIGHT");
                } else {
                    VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_SerialNumber_String, "LHR-OCULUS_LEFT");
                }
        }
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "touchlink");
        VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "TouchLink Tracker");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Ocusus");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_HardwareRevision_String, "14");
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, 14U);
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_GenericTracker);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ResourceRoot_String, "htc");

        if (isWaist)
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, "htc/vive_trackerLHR-OCULUS_VROBJECT");
        else
            vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RegisteredDeviceType_String, (isRightHand)?"htc/vive_trackerLHR-OCULUS_RIGHT": "htc/vive_trackerLHR-OCULUS_LEFT");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
        vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_Invalid);
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ControllerType_String, "vive_tracker");

        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.gif");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_error.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.png");
        vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.png");

        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDisplayComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasDriverDirectModeComponent_Bool, false);
        vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasVirtualDisplayComponent_Bool, false);


        // return a constant that's not 0 (invalid), 1 is reserved for Oculus, so let's use that ;)
        vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 1);

        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/grip/value", &m_compGripv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/button", &m_compGripb);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/touch"    , &m_compGript);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value" , &m_compTrigv, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/trigger/touch" , &m_compTrigt);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x"    , &m_compJoyx, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y"    , &m_compJoyy, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/click", &m_compJoyc);
        vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/touch", &m_compJoyt);

        // create our haptic component
        vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, "/output/haptic", &m_compHaptic);

        if (isRightHand)
        {
            // Transformation inversion along 0YZ plane
            for (size_t i = 1U; i < NUM_BONES; i++)
            {
                right_open_hand_pose[i].position.v[0] *= -1.f;
                right_fist_pose[i].position.v[0] *= -1.f;
                switch (i)
                {
                case HSB_Wrist:
                {
                    right_open_hand_pose[i].orientation.y *= -1.f;
                    right_fist_pose[i].orientation.y *= -1.f;
                    right_open_hand_pose[i].orientation.z *= -1.f;
                    right_fist_pose[i].orientation.z *= -1.f;
                } break;

                case HSB_Thumb0:
                case HSB_IndexFinger0:
                case HSB_MiddleFinger0:
                case HSB_RingFinger0:
                case HSB_PinkyFinger0:
                {
                    right_open_hand_pose[i].orientation.z *= -1.f;
                    right_fist_pose[i].orientation.z *= -1.f;
                    std::swap(right_open_hand_pose[i].orientation.x, right_open_hand_pose[i].orientation.w);
                    std::swap(right_fist_pose[i].orientation.x, right_fist_pose[i].orientation.w);;
                    right_open_hand_pose[i].orientation.w *= -1.f;
                    right_fist_pose[i].orientation.w *= -1.f;
                    std::swap(right_open_hand_pose[i].orientation.y, right_open_hand_pose[i].orientation.z);
                    std::swap(right_fist_pose[i].orientation.y, right_fist_pose[i].orientation.z);
                } break;
                }
            }
        }

        return VRInitError_None;
    }
    enum HandSkeletonBone : size_t
    {
        HSB_Root = 0U,
        HSB_Wrist,
        HSB_Thumb0,
        HSB_Thumb1,
        HSB_Thumb2,
        HSB_Thumb3, // Last, no effect
        HSB_IndexFinger0,
        HSB_IndexFinger1,
        HSB_IndexFinger2,
        HSB_IndexFinger3,
        HSB_IndexFinger4, // Last, no effect
        HSB_MiddleFinger0,
        HSB_MiddleFinger1,
        HSB_MiddleFinger2,
        HSB_MiddleFinger3,
        HSB_MiddleFinger4, // Last, no effect
        HSB_RingFinger0,
        HSB_RingFinger1,
        HSB_RingFinger2,
        HSB_RingFinger3,
        HSB_RingFinger4, // Last, no effect
        HSB_PinkyFinger0,
        HSB_PinkyFinger1,
        HSB_PinkyFinger2,
        HSB_PinkyFinger3,
        HSB_PinkyFinger4, // Last, no effect
        HSB_Aux_Thumb,
        HSB_Aux_IndexFinger,
        HSB_Aux_MiddleFinger,
        HSB_Aux_RingFinger,
        HSB_Aux_PinkyFinger,

        HSB_Count
    };
    virtual void Deactivate()
    {
        m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
    }

    virtual void EnterStandby()
    {
    }

    void* GetComponent(const char* pchComponentNameAndVersion)
    {
        // override this to add a component to a driver
        return NULL;
    }

    virtual void PowerOff()
    {
    }

    /** debug request from a client */
    virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
    {
        if (unResponseBufferSize >= 1)
            pchResponseBuffer[0] = 0;
    }


    ovrQuatf ovrQuatfmul(ovrQuatf q1, ovrQuatf q2) {
        ovrQuatf result = { 0 };
        result.x = q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
        result.y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
        result.z = q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
        result.w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w;
        return result;
    }
    ovrVector3f rotateVector(const ovrVector3f _V, ovrQuatf q)const {
        ovrVector3f vec;   // any constructor will do
        float r, i, j, k;
        r = q.w;
        i = q.x;
        j = q.y;
        k = q.z;
        vec.x = 2 * (r * _V.z * j + i * _V.z * k - r * _V.y * k + i * _V.y * j) + _V.x * (r * r + i * i - j * j - k * k);
        vec.y = 2 * (r * _V.x * k + i * _V.x * j - r * _V.z * i + j * _V.z * k) + _V.y * (r * r - i * i + j * j - k * k);
        vec.z = 2 * (r * _V.y * i - r * _V.x * j + i * _V.x * k + j * _V.y * k) + _V.z * (r * r - i * i - j * j + k * k);
        return vec;
    }

    ovrVector3f crossProduct(const ovrVector3f v, ovrVector3f p) const
               {
        return ovrVector3f{ v.y * p.z - v.z * p.y, v.z * p.x - v.x * p.z, v.x * p.y - v.y * p.x };
               }

    ovrVector3f rotateVector2 ( ovrVector3f v, ovrQuatf q)
        {
            // nVidia SDK implementation
        ovrVector3f uv, uuv;
        ovrVector3f qvec{ q.x, q.y, q.z };
            uv = crossProduct(qvec,v);
            uuv = crossProduct(qvec,uv);
            uv.x *= (2.0f * q.w);
            uv.y *= (2.0f * q.w);
            uv.z *= (2.0f * q.w);
            uuv.x *= 2.0f;
            uuv.y *= 2.0f;
            uuv.z *= 2.0f;
        
            return ovrVector3f{ v.x + uv.x + uuv.x, v.y + uv.y + uuv.y, v.z + uv.z + uuv.z};
        }
    virtual DriverPose_t GetPose()
    {
        m_last_pose.poseTimeOffset = m_time_of_last_pose - ovr_GetTimeInSeconds();
        return this->m_last_pose;
    }
    virtual DriverPose_t CalculatePose()
    {
        ovrTrackingState ss = ovr_GetTrackingState(mSession, 0, false);
        ovrPoseStatef poseState;

        if (isWaist){
            ovrTrackedDeviceType type = ovrTrackedDevice_Object0;
            ovr_GetDevicePoses(mSession, &type, 1, 0, &poseState);
        } else {
            poseState = ss.HandPoses[isRightHand];
        }

        m_time_of_last_pose = poseState.TimeInSeconds;
        DriverPose_t pose = { 0 };
        pose.poseIsValid = true;
        pose.result = TrackingResult_Running_OK;
        pose.deviceIsConnected = true;
    
        ovrQuatf hand_qoffset = { 0.3420201, 0, 0, 0.9396926 };
        ovrQuatf hand_input = poseState.ThePose.Orientation;
        ovrQuatf hand_result = ovrQuatfmul(hand_input, hand_qoffset);
        ovrVector3f hand_voffset={ 0,0,0 };
        if (isRightHand) {
            hand_voffset = rotateVector2(hand_offset, hand_input);
        } else {
            ovrVector3f left_hand_offset = hand_offset;
            left_hand_offset.x = -left_hand_offset.x;
            hand_voffset = rotateVector2(left_hand_offset, hand_input);
        }
    
        pose.qRotation.w = hand_result.w;
        pose.qRotation.x = hand_result.x;
        pose.qRotation.y = hand_result.y;
        pose.qRotation.z = hand_result.z;
        ovrVector3f position;
        position.x = poseState.ThePose.Position.x + hand_voffset.x + hand_offset2.x;
        position.y = poseState.ThePose.Position.y + hand_voffset.y + hand_offset2.y;
        position.z = poseState.ThePose.Position.z + hand_voffset.z + hand_offset2.z;
        
        pose.vecPosition[0] = position.x;
        pose.vecPosition[1] = position.y;
        pose.vecPosition[2] = position.z;

        ovrVector3f linAcc = (poseState.LinearAcceleration);
        ovrVector3f linVel = (poseState.LinearVelocity);
        ovrQuatf hand_nqoffset = { 0.3420201, 0, 0, -0.9396926 };

        pose.vecAcceleration[0] = linAcc.x;
        pose.vecAcceleration[1] = linAcc.y;
        pose.vecAcceleration[2] = linAcc.z;

        pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
        pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

        pose.vecVelocity[0] = linVel.x;
        pose.vecVelocity[1] = linVel.y;
        pose.vecVelocity[2] = linVel.z;

        pose.vecAngularVelocity[0] = poseState.AngularVelocity.x;
        pose.vecAngularVelocity[1] = poseState.AngularVelocity.y;
        pose.vecAngularVelocity[2] = poseState.AngularVelocity.z;

        pose.poseTimeOffset = -0.01;
        return pose;
    }
                          
    const static int NUM_BONES = 31;

    const VRBoneTransform_t left_open_hand_pose[NUM_BONES] = {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.012083f,  0.028070f,  0.025050f,  1.000000f}, { 0.464112f,  0.567418f,  0.272106f,  0.623374f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.994838f,  0.082939f,  0.019454f,  0.055130f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.974793f, -0.003213f,  0.021867f, -0.222015f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.000632f,  0.026866f,  0.015002f,  1.000000f}, { 0.644251f,  0.421979f, -0.478202f,  0.422133f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.995332f,  0.007007f, -0.039124f,  0.087949f} },
    { { 0.043930f, -0.000000f, -0.000000f,  1.000000f}, { 0.997891f,  0.045808f,  0.002142f, -0.045943f} },
    { { 0.028695f,  0.000000f,  0.000000f,  1.000000f}, { 0.999649f,  0.001850f, -0.022782f, -0.013409f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.002177f,  0.007120f,  0.016319f,  1.000000f}, { 0.546723f,  0.541276f, -0.442520f,  0.460749f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.980294f, -0.167261f, -0.078959f,  0.069368f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.997947f,  0.018493f,  0.013192f,  0.059886f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.997394f, -0.003328f, -0.028225f, -0.066315f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.000513f, -0.006545f,  0.016348f,  1.000000f}, { 0.516692f,  0.550143f, -0.495548f,  0.429888f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.990420f, -0.058696f, -0.101820f,  0.072495f} },
    { { 0.040697f,  0.000000f,  0.000000f,  1.000000f}, { 0.999545f, -0.002240f,  0.000004f,  0.030081f} },
    { { 0.028747f, -0.000000f, -0.000000f,  1.000000f}, { 0.999102f, -0.000721f, -0.012693f,  0.040420f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.002478f, -0.018981f,  0.015214f,  1.000000f}, { 0.526918f,  0.523940f, -0.584025f,  0.326740f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.986609f, -0.059615f, -0.135163f,  0.069132f} },
    { { 0.030220f,  0.000000f,  0.000000f,  1.000000f}, { 0.994317f,  0.001896f, -0.000132f,  0.106446f} },
    { { 0.018187f,  0.000000f,  0.000000f,  1.000000f}, { 0.995931f, -0.002010f, -0.052079f, -0.073526f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.006059f,  0.056285f,  0.060064f,  1.000000f}, { 0.737238f,  0.202745f,  0.594267f,  0.249441f} },
    { {-0.040416f, -0.043018f,  0.019345f,  1.000000f}, {-0.290331f,  0.623527f, -0.663809f, -0.293734f} },
    { {-0.039354f, -0.075674f,  0.047048f,  1.000000f}, {-0.187047f,  0.678062f, -0.659285f, -0.265683f} },
    { {-0.038340f, -0.090987f,  0.082579f,  1.000000f}, {-0.183037f,  0.736793f, -0.634757f, -0.143936f} },
    { {-0.031806f, -0.087214f,  0.121015f,  1.000000f}, {-0.003659f,  0.758407f, -0.639342f, -0.126678f} },
    };

    const VRBoneTransform_t left_fist_pose[NUM_BONES] =
    {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.016305f,  0.027529f,  0.017800f,  1.000000f}, { 0.225703f,  0.483332f,  0.126413f,  0.836342f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.894335f, -0.013302f, -0.082902f,  0.439448f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.842428f,  0.000655f,  0.001244f,  0.538807f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.003802f,  0.021514f,  0.012803f,  1.000000f}, { 0.617314f,  0.395175f, -0.510874f,  0.449185f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.737291f, -0.032006f, -0.115013f,  0.664944f} },
    { { 0.043287f, -0.000000f, -0.000000f,  1.000000f}, { 0.611381f,  0.003287f,  0.003823f,  0.791321f} },
    { { 0.028275f,  0.000000f,  0.000000f,  1.000000f}, { 0.745388f, -0.000684f, -0.000945f,  0.666629f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.005787f,  0.006806f,  0.016534f,  1.000000f}, { 0.514203f,  0.522315f, -0.478348f,  0.483700f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.723653f, -0.097901f,  0.048546f,  0.681458f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.637464f, -0.002366f, -0.002831f,  0.770472f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.658008f,  0.002610f,  0.003196f,  0.753000f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.004123f, -0.006858f,  0.016563f,  1.000000f}, { 0.489609f,  0.523374f, -0.520644f,  0.463997f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.759970f, -0.055609f,  0.011571f,  0.647471f} },
    { { 0.040331f,  0.000000f,  0.000000f,  1.000000f}, { 0.664315f,  0.001595f,  0.001967f,  0.747449f} },
    { { 0.028489f, -0.000000f, -0.000000f,  1.000000f}, { 0.626957f, -0.002784f, -0.003234f,  0.779042f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.001131f, -0.019295f,  0.015429f,  1.000000f}, { 0.479766f,  0.477833f, -0.630198f,  0.379934f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.827001f,  0.034282f,  0.003440f,  0.561144f} },
    { { 0.029874f,  0.000000f,  0.000000f,  1.000000f}, { 0.702185f, -0.006716f, -0.009289f,  0.711903f} },
    { { 0.017979f,  0.000000f,  0.000000f,  1.000000f}, { 0.676853f,  0.007956f,  0.009917f,  0.736009f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.019716f,  0.002802f,  0.093937f,  1.000000f}, { 0.377286f, -0.540831f,  0.150446f, -0.736562f} },
    { { 0.000171f,  0.016473f,  0.096515f,  1.000000f}, {-0.006456f,  0.022747f, -0.932927f, -0.359287f} },
    { { 0.000448f,  0.001536f,  0.116543f,  1.000000f}, {-0.039357f,  0.105143f, -0.928833f, -0.353079f} },
    { { 0.003949f, -0.014869f,  0.130608f,  1.000000f}, {-0.055071f,  0.068695f, -0.944016f, -0.317933f} },
    { { 0.003263f, -0.034685f,  0.139926f,  1.000000f}, { 0.019690f, -0.100741f, -0.957331f, -0.270149f} },
    };

    VRBoneTransform_t right_open_hand_pose[NUM_BONES] = {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.012083f,  0.028070f,  0.025050f,  1.000000f}, { 0.464112f,  0.567418f,  0.272106f,  0.623374f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.994838f,  0.082939f,  0.019454f,  0.055130f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.974793f, -0.003213f,  0.021867f, -0.222015f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.000632f,  0.026866f,  0.015002f,  1.000000f}, { 0.644251f,  0.421979f, -0.478202f,  0.422133f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.995332f,  0.007007f, -0.039124f,  0.087949f} },
    { { 0.043930f, -0.000000f, -0.000000f,  1.000000f}, { 0.997891f,  0.045808f,  0.002142f, -0.045943f} },
    { { 0.028695f,  0.000000f,  0.000000f,  1.000000f}, { 0.999649f,  0.001850f, -0.022782f, -0.013409f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.002177f,  0.007120f,  0.016319f,  1.000000f}, { 0.546723f,  0.541276f, -0.442520f,  0.460749f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.980294f, -0.167261f, -0.078959f,  0.069368f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.997947f,  0.018493f,  0.013192f,  0.059886f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.997394f, -0.003328f, -0.028225f, -0.066315f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.000513f, -0.006545f,  0.016348f,  1.000000f}, { 0.516692f,  0.550143f, -0.495548f,  0.429888f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.990420f, -0.058696f, -0.101820f,  0.072495f} },
    { { 0.040697f,  0.000000f,  0.000000f,  1.000000f}, { 0.999545f, -0.002240f,  0.000004f,  0.030081f} },
    { { 0.028747f, -0.000000f, -0.000000f,  1.000000f}, { 0.999102f, -0.000721f, -0.012693f,  0.040420f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.002478f, -0.018981f,  0.015214f,  1.000000f}, { 0.526918f,  0.523940f, -0.584025f,  0.326740f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.986609f, -0.059615f, -0.135163f,  0.069132f} },
    { { 0.030220f,  0.000000f,  0.000000f,  1.000000f}, { 0.994317f,  0.001896f, -0.000132f,  0.106446f} },
    { { 0.018187f,  0.000000f,  0.000000f,  1.000000f}, { 0.995931f, -0.002010f, -0.052079f, -0.073526f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { {-0.006059f,  0.056285f,  0.060064f,  1.000000f}, { 0.737238f,  0.202745f,  0.594267f,  0.249441f} },
    { {-0.040416f, -0.043018f,  0.019345f,  1.000000f}, {-0.290331f,  0.623527f, -0.663809f, -0.293734f} },
    { {-0.039354f, -0.075674f,  0.047048f,  1.000000f}, {-0.187047f,  0.678062f, -0.659285f, -0.265683f} },
    { {-0.038340f, -0.090987f,  0.082579f,  1.000000f}, {-0.183037f,  0.736793f, -0.634757f, -0.143936f} },
    { {-0.031806f, -0.087214f,  0.121015f,  1.000000f}, {-0.003659f,  0.758407f, -0.639342f, -0.126678f} },
    };

    VRBoneTransform_t right_fist_pose[NUM_BONES] =
    {
    { { 0.000000f,  0.000000f,  0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { {-0.034038f,  0.036503f,  0.164722f,  1.000000f}, {-0.055147f, -0.078608f, -0.920279f,  0.379296f} },
    { {-0.016305f,  0.027529f,  0.017800f,  1.000000f}, { 0.225703f,  0.483332f,  0.126413f,  0.836342f} },
    { { 0.040406f,  0.000000f, -0.000000f,  1.000000f}, { 0.894335f, -0.013302f, -0.082902f,  0.439448f} },
    { { 0.032517f,  0.000000f,  0.000000f,  1.000000f}, { 0.842428f,  0.000655f,  0.001244f,  0.538807f} },
    { { 0.030464f, -0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f, -0.000000f,  0.000000f} },
    { { 0.003802f,  0.021514f,  0.012803f,  1.000000f}, { 0.617314f,  0.395175f, -0.510874f,  0.449185f} },
    { { 0.074204f, -0.005002f,  0.000234f,  1.000000f}, { 0.737291f, -0.032006f, -0.115013f,  0.664944f} },
    { { 0.043287f, -0.000000f, -0.000000f,  1.000000f}, { 0.611381f,  0.003287f,  0.003823f,  0.791321f} },
    { { 0.028275f,  0.000000f,  0.000000f,  1.000000f}, { 0.745388f, -0.000684f, -0.000945f,  0.666629f} },
    { { 0.022821f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f, -0.000000f,  0.000000f, -0.000000f} },
    { { 0.005787f,  0.006806f,  0.016534f,  1.000000f}, { 0.514203f,  0.522315f, -0.478348f,  0.483700f} },
    { { 0.070953f,  0.000779f,  0.000997f,  1.000000f}, { 0.723653f, -0.097901f,  0.048546f,  0.681458f} },
    { { 0.043108f,  0.000000f,  0.000000f,  1.000000f}, { 0.637464f, -0.002366f, -0.002831f,  0.770472f} },
    { { 0.033266f,  0.000000f,  0.000000f,  1.000000f}, { 0.658008f,  0.002610f,  0.003196f,  0.753000f} },
    { { 0.025892f, -0.000000f,  0.000000f,  1.000000f}, { 0.999195f, -0.000000f,  0.000000f,  0.040126f} },
    { { 0.004123f, -0.006858f,  0.016563f,  1.000000f}, { 0.489609f,  0.523374f, -0.520644f,  0.463997f} },
    { { 0.065876f,  0.001786f,  0.000693f,  1.000000f}, { 0.759970f, -0.055609f,  0.011571f,  0.647471f} },
    { { 0.040331f,  0.000000f,  0.000000f,  1.000000f}, { 0.664315f,  0.001595f,  0.001967f,  0.747449f} },
    { { 0.028489f, -0.000000f, -0.000000f,  1.000000f}, { 0.626957f, -0.002784f, -0.003234f,  0.779042f} },
    { { 0.022430f, -0.000000f,  0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.001131f, -0.019295f,  0.015429f,  1.000000f}, { 0.479766f,  0.477833f, -0.630198f,  0.379934f} },
    { { 0.062878f,  0.002844f,  0.000332f,  1.000000f}, { 0.827001f,  0.034282f,  0.003440f,  0.561144f} },
    { { 0.029874f,  0.000000f,  0.000000f,  1.000000f}, { 0.702185f, -0.006716f, -0.009289f,  0.711903f} },
    { { 0.017979f,  0.000000f,  0.000000f,  1.000000f}, { 0.676853f,  0.007956f,  0.009917f,  0.736009f} },
    { { 0.018018f,  0.000000f, -0.000000f,  1.000000f}, { 1.000000f,  0.000000f,  0.000000f,  0.000000f} },
    { { 0.019716f,  0.002802f,  0.093937f,  1.000000f}, { 0.377286f, -0.540831f,  0.150446f, -0.736562f} },
    { { 0.000171f,  0.016473f,  0.096515f,  1.000000f}, {-0.006456f,  0.022747f, -0.932927f, -0.359287f} },
    { { 0.000448f,  0.001536f,  0.116543f,  1.000000f}, {-0.039357f,  0.105143f, -0.928833f, -0.353079f} },
    { { 0.003949f, -0.014869f,  0.130608f,  1.000000f}, {-0.055071f,  0.068695f, -0.944016f, -0.317933f} },
    { { 0.003263f, -0.034685f,  0.139926f,  1.000000f}, { 0.019690f, -0.100741f, -0.957331f, -0.270149f} },
    };

    void RunFrame()
    {
        m_last_pose = this->CalculatePose();
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_last_pose, sizeof(DriverPose_t));
    }

    void ProcessEvent(const vr::VREvent_t& vrEvent)
    {
        switch (vrEvent.eventType)
        {
        case vr::VREvent_Input_HapticVibration:
        {
            if (vrEvent.data.hapticVibration.componentHandle == m_compHaptic)
            {
#if USE_MUTEX                
                if (!WaitForSingleObject(comm_mutex, 10)) {
#endif
                    comm_buffer->vib_duration_s[isRightHand] = vrEvent.data.hapticVibration.fDurationSeconds;
                    comm_buffer->vib_amplitude[isRightHand] = vrEvent.data.hapticVibration.fAmplitude;
                    comm_buffer->vib_frequency[isRightHand] = vrEvent.data.hapticVibration.fFrequency;
                    comm_buffer->vib_valid[isRightHand] = true;
#if USE_MUTEX
                    ReleaseMutex(comm_mutex);
                }
#endif
            }
        }
        break;
        }
    }

    std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
    vr::TrackedDeviceIndex_t m_unObjectId;
    vr::PropertyContainerHandle_t m_ulPropertyContainer;

    vr::VRInputComponentHandle_t m_compAc;
    vr::VRInputComponentHandle_t m_compBc;
    vr::VRInputComponentHandle_t m_compXc;
    vr::VRInputComponentHandle_t m_compYc;
    vr::VRInputComponentHandle_t m_compAt;
    vr::VRInputComponentHandle_t m_compBt;
    vr::VRInputComponentHandle_t m_compXt;
    vr::VRInputComponentHandle_t m_compYt;
    vr::VRInputComponentHandle_t m_compTrigv;
    vr::VRInputComponentHandle_t m_compTrigt;
    vr::VRInputComponentHandle_t m_compGripv;
    vr::VRInputComponentHandle_t m_compGripb;
    vr::VRInputComponentHandle_t m_compGript;
    vr::VRInputComponentHandle_t m_compJoyx;
    vr::VRInputComponentHandle_t m_compJoyy;
    vr::VRInputComponentHandle_t m_compJoyc;
    vr::VRInputComponentHandle_t m_compJoyt;
    vr::VRInputComponentHandle_t m_compSysc;
    vr::VRInputComponentHandle_t m_compSkel;

    vr::VRInputComponentHandle_t m_compHaptic;
    shared_buffer* comm_buffer;
    std::string m_sSerialNumber;
    std::string m_sModelNumber;
    ovrSession mSession;
    bool isRightHand;
    bool isWaist;
    ovrVector3f hand_offset;
    ovrVector3f hand_offset2;

    DriverPose_t m_last_pose;
    float m_time_of_last_pose;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_OVRTL : public IServerTrackedDeviceProvider
{
public:
    virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
    virtual void Cleanup();
    virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
    virtual void RunFrame();
    virtual bool ShouldBlockStandbyMode() { return false; }
    virtual void EnterStandby() {}
    virtual void LeaveStandby() {}

private:
    void InitRenderTargets(const ovrHmdDesc& hmdDesc);
    void  Render();
    ovrSession mSession = nullptr;
    ovrGraphicsLuid luid{};
    CSampleControllerDriver* m_pLController = nullptr;
    CSampleControllerDriver* m_pRController = nullptr;
    CSampleControllerDriver* m_pWaistController = nullptr;
    HANDLE hMapFile;
    shared_buffer* comm_buffer;
};

CServerDriver_OVRTL g_serverDriverNull;


void CServerDriver_OVRTL::Render()
{
}
void CServerDriver_OVRTL::InitRenderTargets(const ovrHmdDesc& hmdDesc)
{
}
ovrQuatf ToQuaternion(double x, double y, double z) 
{
    // Abbreviations for the various angular functions
    double cx = cos(z * 0.5);
    double sx = sin(z * 0.5);
    double cy = cos(x * 0.5);
    double sy = sin(x * 0.5);
    double cz = cos(y * 0.5);
    double sz = sin(y * 0.5);

    ovrQuatf q;
    q.w = cz * cy * cx + sz * sy * sx;
    q.x = sz * cy * cx - cz * sy * sx;
    q.y = cz * sy * cx + sz * cy * sx;
    q.z = cz * cy * sx - sz * sy * cx;

    return q;
}

EVRInitError CServerDriver_OVRTL::Init(vr::IVRDriverContext* pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    InitDriverLog(vr::VRDriverLog());

    auto appdata = std::filesystem::temp_directory_path()
        .parent_path()
        .parent_path();

    appdata /= "TouchLink";

    if (!std::filesystem::exists(appdata))
        std::filesystem::create_directories(appdata);

    // return of the konct
    std::filesystem::path noHip         = std::filesystem::absolute( appdata / std::filesystem::path( "no-hip" ));
    std::filesystem::path noLeftLeg     = std::filesystem::absolute( appdata / std::filesystem::path( "no-left-leg" ));
    std::filesystem::path noRightLeg    = std::filesystem::absolute( appdata / std::filesystem::path( "no-right-leg" ));
    std::filesystem::path bro           = std::filesystem::absolute( appdata / std::filesystem::path( "no-bitches" ));

    s_spawnHipTracker = !(std::filesystem::exists( noHip ));
    s_spawnRightLegTracker = !(std::filesystem::exists( noLeftLeg ));
    s_spawnLeftLegTracker = !(std::filesystem::exists( noRightLeg ));
    auto broSus = !(std::filesystem::exists( bro ));

    DriverLog(("noHipFile:: " + noHip.string()).c_str());
    DriverLog(("noLeftLeg:: " + noLeftLeg.string()).c_str());
    DriverLog(("noRightLeg:: " + noRightLeg.string()).c_str());
    if (broSus)
    DriverLog("bro i tought you had bitches :(");

    //map the shared memory buffer for communicating button and vibration data

#if USE_SHARE_MEM_BUFFER 1    
   
    hMapFile = OpenFileMapping(
        FILE_MAP_ALL_ACCESS,   // read/write access
        FALSE,                 // do not inherit the name
        L"Local\\oculus_steamvr_touch_controller_data_channel");               // name of mapping object

    if (hMapFile == NULL)
    {
        std::cout<<"Could not create file mapping object "<< GetLastError() << std::endl;
        return VRInitError_Init_Internal;
    }
    comm_buffer = (shared_buffer*)MapViewOfFile(hMapFile,   // handle to map object
        FILE_MAP_ALL_ACCESS, // read/write permission
        0,
        0,
        sizeof(shared_buffer));

    if (comm_buffer == NULL)
    {
        std::cout << "Could not map view of file" << GetLastError() << std::endl;

        CloseHandle(hMapFile);

        return VRInitError_Init_Internal;
    }
#endif

    // get a handle to the inter-process mutex used for accessing the shared data structure
#if USE_MUTEX
    comm_mutex = CreateMutex(0, false, L"Local\\oculus_steamvr_touch_controller_mutex");
    if (comm_mutex == NULL)
    {
        std::cout << "Could notopen mutex" << GetLastError() << std::endl;

        return VRInitError_Init_Internal;
    }
#endif


    if (!mSession) {
        ovrInitParams initParams = { ovrInit_RequestVersion | ovrInit_FocusAware | ovrInit_Invisible, OVR_MINOR_VERSION, NULL, 0, 0 };
        if (OVR_FAILURE(ovr_Initialize(&initParams)))
            return VRInitError_Init_Internal;

        if (OVR_FAILURE(ovr_Create(&mSession, &luid)))
            return VRInitError_Init_Internal;

        if (OVR_FAILURE(ovr_SetTrackingOriginType(mSession, ovrTrackingOrigin_FloorLevel)))
            return VRInitError_Init_Internal;
    }
#if CREATE_CONTROLLERS

    if (s_spawnLeftLegTracker) {
        m_pLController = new CSampleControllerDriver(mSession, false, false, comm_buffer/*, overall_offset, overall_rotation*/);
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_pLController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pLController);
    }

    if (s_spawnRightLegTracker) {
        m_pRController = new CSampleControllerDriver(mSession, true, false, comm_buffer/*, overall_offset, overall_rotation*/);
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_pRController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pRController);
    }
    
    if (s_spawnHipTracker) {
        m_pWaistController = new CSampleControllerDriver(mSession, false, true, comm_buffer/*, overall_offset, overall_rotation*/);
        vr::VRServerDriverHost()->TrackedDeviceAdded(m_pWaistController->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pWaistController);
    }
#endif

    return VRInitError_None;
}

void CServerDriver_OVRTL::Cleanup()
{
    CleanupDriverLog();
#if CREATE_CONTROLLERS
    delete m_pLController;
    m_pLController = NULL;
    delete m_pRController;
    m_pRController = NULL;
    delete m_pWaistController;
    m_pWaistController = NULL;
#endif
    ovr_Destroy(mSession);
    ovr_Shutdown();
#if USE_SHARE_MEM_BUFFER
    UnmapViewOfFile(comm_buffer);

    CloseHandle(hMapFile);
#endif
#if USE_MUTEX
    CloseHandle(comm_mutex);
#endif
}


void CServerDriver_OVRTL::RunFrame()
{
    if (m_pLController)
    {
        m_pLController->RunFrame();
    }
    if (m_pRController)
    {
        m_pRController->RunFrame();
    }
    if (m_pWaistController)
    {
        m_pWaistController->RunFrame();
    }

    vr::VREvent_t vrEvent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
    {
#if USE_SHARE_MEM_BUFFER
#if USE_MUTEX
        if (!WaitForSingleObject(comm_mutex, 100)) {
            comm_buffer->vrEvent_type = vrEvent.eventType;
            ReleaseMutex(comm_mutex);
        }
#else
        comm_buffer->vrEvent_type = vrEvent.eventType;
#endif
#endif

        if (m_pLController)
        {
            m_pLController->ProcessEvent(vrEvent);
        }
        if (m_pRController)
        {
            m_pRController->ProcessEvent(vrEvent);
        }
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
    if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
    {
        return &g_serverDriverNull;
    }
    if (0 == strcmp(IVRWatchdogProvider_Version, pInterfaceName))
    {
        return &g_watchdogDriverNull;
    }

    if (pReturnCode)
        *pReturnCode = VRInitError_Init_InterfaceNotFound;

    return NULL;
}
