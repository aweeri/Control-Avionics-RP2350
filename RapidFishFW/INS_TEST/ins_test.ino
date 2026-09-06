/*
 * =============================================================================
 *  INS Test Firmware — Barebones ESKF for CARP-LITE
 * =============================================================================
 *  Standalone test firmware that runs an Error-State Kalman Filter on core 1,
 *  fusing IMU + baro + GPS into a continuous 3D state estimate.
 *
 *  No flight state machine, no recovery logic — just sensor fusion.
 *  Telemetry is transmitted via APID 0 and APID 1 frames (existing formats),
 *  so the existing ground station works with zero changes.
 *
 *  The fused gravity vector goes into accel.x/y/z in APID 0, so the
 *  ground station's 3D attitude visualization shows smooth, accurate
 *  orientation automatically.
 * =============================================================================
 */

#include <pico.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/critical_section.h"
#include <Adafruit_LSM6DSO32.h>
#include "SparkFun_LSM6DSV16X.h"
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <RadioLib.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <string.h>

// =============================================================================
// [1] CONFIGURATION
// =============================================================================

#define LED_PIN         25
#define NUM_LEDS        1

#define LSM_CS_PIN      17    // IMU chip select (SPI0)
#define BMP_SDA_PIN     4     // Barometer I2C SDA
#define BMP_SCL_PIN     5     // Barometer I2C SCL

#define BAT_ADC_PIN     26    // Battery voltage divider ADC

#define RADIO_CS_PIN    13
#define RADIO_IRQ_PIN   6
#define RADIO_RST_PIN   11
#define RADIO_BUSY_PIN  10

#define RADIO_FREQUENCY_MHZ 434.0f
#define RADIO_TX_INTERVAL_MS 80
#define RADIO_POWER_DBM      22.0f

#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUD   115200

const uint32_t FIRMWARE_RESERVED_SIZE = 2 * 1024 * 1024;
const uint32_t FLIGHT_DATA_FLASH_SIZE = 14 * 1024 * 1024;
const uint32_t SYNC_WORD = 0x1ACFFC1D;

// =============================================================================
// [2] QUATERNION MATH LIBRARY
// =============================================================================

struct Quat {
    float w, x, y, z;

    Quat() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quat(float _w, float _x, float _y, float _z) : w(_w), x(_x), y(_y), z(_z) {}

    static Quat identity() { return Quat(1.0f, 0.0f, 0.0f, 0.0f); }

    float normSq() const { return w*w + x*x + y*y + z*z; }
    float norm() const { return sqrtf(normSq()); }

    void normalize() {
        float n = norm();
        if (n > 1e-10f) { float inv = 1.0f / n; w *= inv; x *= inv; y *= inv; z *= inv; }
        else { w = 1.0f; x = 0.0f; y = 0.0f; z = 0.0f; }
    }

    Quat normalized() const { Quat q = *this; q.normalize(); return q; }
    Quat conjugated() const { return Quat(w, -x, -y, -z); }

    Quat operator*(const Quat& r) const {
        return Quat(
            w * r.w - x * r.x - y * r.y - z * r.z,
            w * r.x + x * r.w + y * r.z - z * r.y,
            w * r.y - x * r.z + y * r.w + z * r.x,
            w * r.z + x * r.y - y * r.x + z * r.w
        );
    }

    // Rotate vector body → world: v' = q * v * q_conj
    void rotateVector(const float v[3], float out[3]) const {
        float qx = x, qy = y, qz = z;
        float c1 = qy * v[2] - qz * v[1];
        float c2 = qz * v[0] - qx * v[2];
        float c3 = qx * v[1] - qy * v[0];
        float t1 = c1 + w * v[0], t2 = c2 + w * v[1], t3 = c3 + w * v[2];
        float d1 = qy * t3 - qz * t2, d2 = qz * t1 - qx * t3, d3 = qx * t2 - qy * t1;
        out[0] = v[0] + 2.0f * d1; out[1] = v[1] + 2.0f * d2; out[2] = v[2] + 2.0f * d3;
    }

    // Rotate vector world → body (inverse)
    void rotateVectorInverse(const float v[3], float out[3]) const {
        conjugated().rotateVector(v, out);
    }

    // Axis-angle → quaternion
    static Quat fromAxisAngle(const float axis[3]) {
        float angle = sqrtf(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
        if (angle < 1e-10f) return identity();
        float half = angle * 0.5f;
        float s = sinf(half) / angle;
        return Quat(cosf(half), axis[0] * s, axis[1] * s, axis[2] * s);
    }

    // Quaternion → axis-angle
    void toAxisAngle(float out[3]) const {
        float half = acosf(fmaxf(-1.0f, fminf(1.0f, w)));
        float sin_half = sinf(half);
        if (sin_half < 1e-10f) { out[0] = 0; out[1] = 0; out[2] = 0; }
        else { float scale = 2.0f * half / sin_half; out[0] = x * scale; out[1] = y * scale; out[2] = z * scale; }
    }

    // Rotation matrix (column-major, body → world)
    void toRotationMatrix(float R[9]) const {
        float qw2 = w*w, qx2 = x*x, qy2 = y*y, qz2 = z*z;
        R[0] = qw2 + qx2 - qy2 - qz2;
        R[1] = 2.0f * (x*y + w*z);
        R[2] = 2.0f * (x*z - w*y);
        R[3] = 2.0f * (x*y - w*z);
        R[4] = qw2 - qx2 + qy2 - qz2;
        R[5] = 2.0f * (y*z + w*x);
        R[6] = 2.0f * (x*z + w*y);
        R[7] = 2.0f * (y*z - w*x);
        R[8] = qw2 - qx2 - qy2 + qz2;
    }

    // Rotation that aligns 'from' to 'to'
    static Quat fromTwoVectors(const float from[3], const float to[3]) {
        float fn = sqrtf(from[0]*from[0] + from[1]*from[1] + from[2]*from[2]);
        float tn = sqrtf(to[0]*to[0] + to[1]*to[1] + to[2]*to[2]);
        if (fn < 1e-10f || tn < 1e-10f) return identity();
        float f[3] = {from[0]/fn, from[1]/fn, from[2]/fn};
        float t[3] = {to[0]/tn, to[1]/tn, to[2]/tn};
        float dot = f[0]*t[0] + f[1]*t[1] + f[2]*t[2];
        float cr[3] = {f[1]*t[2] - f[2]*t[1], f[2]*t[0] - f[0]*t[2], f[0]*t[1] - f[1]*t[0]};
        if (dot > 0.99999f) return identity();
        if (dot < -0.99999f) {
            float axis[3] = {1,0,0};
            if (fabsf(f[0]) < 0.9f) { axis[0] = f[1]; axis[1] = -f[0]; axis[2] = 0; }
            else { axis[0] = 0; axis[1] = f[2]; axis[2] = -f[1]; }
            float an = sqrtf(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
            if (an > 1e-10f) { axis[0] /= an; axis[1] /= an; axis[2] /= an; }
            return Quat(0, axis[0], axis[1], axis[2]);
        }
        float s = sqrtf(2.0f * (1.0f + dot));
        return Quat(s * 0.5f, cr[0] / s, cr[1] / s, cr[2] / s);
    }

    // Small-angle update: q ← q ⊗ exp(δθ/2)
    static Quat smallAngleUpdate(const Quat& q, const float dtheta[3]) {
        float half[3] = {dtheta[0]*0.5f, dtheta[1]*0.5f, dtheta[2]*0.5f};
        return (q * fromAxisAngle(half)).normalized();
    }
};

// --- Vector ops ---
inline void v3_add(const float a[3], const float b[3], float o[3]) { o[0]=a[0]+b[0]; o[1]=a[1]+b[1]; o[2]=a[2]+b[2]; }
inline void v3_sub(const float a[3], const float b[3], float o[3]) { o[0]=a[0]-b[0]; o[1]=a[1]-b[1]; o[2]=a[2]-b[2]; }
inline void v3_scale(const float v[3], float s, float o[3]) { o[0]=v[0]*s; o[1]=v[1]*s; o[2]=v[2]*s; }
inline float v3_dot(const float a[3], const float b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline void v3_cross(const float a[3], const float b[3], float o[3]) { o[0]=a[1]*b[2]-a[2]*b[1]; o[1]=a[2]*b[0]-a[0]*b[2]; o[2]=a[0]*b[1]-a[1]*b[0]; }
inline float v3_norm(const float v[3]) { return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
inline void v3_cp(const float s[3], float d[3]) { d[0]=s[0]; d[1]=s[1]; d[2]=s[2]; }

inline void skew3(const float v[3], float o[9]) {
    o[0]=0; o[3]=-v[2]; o[6]=v[1]; o[1]=v[2]; o[4]=0; o[7]=-v[0]; o[2]=-v[1]; o[5]=v[0]; o[8]=0;
}
inline void m3v3(const float M[9], const float v[3], float o[3]) {
    o[0]=M[0]*v[0]+M[3]*v[1]+M[6]*v[2]; o[1]=M[1]*v[0]+M[4]*v[1]+M[7]*v[2]; o[2]=M[2]*v[0]+M[5]*v[1]+M[8]*v[2];
}
inline void m3add(const float A[9], const float B[9], float o[9]) { for (int i=0;i<9;i++) o[i]=A[i]+B[i]; }
inline void m3id(float o[9]) { memset(o,0,36); o[0]=1; o[4]=1; o[8]=1; }

// =============================================================================
// [3] ERROR-STATE KALMAN FILTER
// =============================================================================

class ESKF {
public:
    ESKF();

    // Configuration (set before init)
    float sigma_gyro;          // rad/s/√Hz
    float sigma_accel;         // m/s²/√Hz
    float sigma_gyro_bias;     // rad/s²/√Hz
    float sigma_accel_bias;    // m/s³/√Hz
    float sigma_accel_meas;    // m/s²
    float sigma_baro_meas;     // m
    float sigma_gps_pos;       // m
    float sigma_gps_vel;       // m/s
    float boost_accel_threshold;   // m/s²
    float outlier_threshold_sigmas;

    void init(float ax, float ay, float az, float baro_altitude);
    void predict(const float gyro[3], const float accel[3], float dt);
    void correctAccel(const float accel[3]);
    void correctBaro(float altitude);
    void correctGPS(const float pos[3], const float vel[3], float pos_noise, float vel_noise);

    struct FusedState {
        float quat_w, quat_x, quat_y, quat_z;
        float pos_n, pos_e, pos_d;
        float vel_n, vel_e, vel_d;
        float altitude;
        float gyro_bias_x, gyro_bias_y, gyro_bias_z;
        float accel_bias_x, accel_bias_y, accel_bias_z;
        float corrected_accel[3];
        float corrected_gyro[3];
        float corrected_pressure;
        float confidence;
        float pos_uncertainty;
        float att_uncertainty;
        uint8_t sensor_status;
        uint32_t timestamp_us;
    };

    void getFusedState(FusedState& out, uint32_t timestamp_us);
    float getInnovationMagnitude() const { return last_innovation_mag; }
    bool isConverged() const { return convergence_count > 100; }

private:
    static const int N = 15;
    static const float GRAVITY_NED[3];

    Quat q;
    float p[3], v[3];
    float bg[3], ba[3];
    float P[N * N];

    float last_innovation_mag;
    int correction_count, convergence_count;
    float reference_altitude;

    void computeF(const float gyro_c[3], const float accel_c[3], float dt, float F[225]);
    void computeQ(float dt, float Q[225]);
    bool kalmanUpdate(const float* H, const float* z, const float* R, int m);
    bool isOutlier(const float* H, const float* z, const float* R, int m);
    void injectErrorState(const float dx[15]);
};

const float ESKF::GRAVITY_NED[3] = {0.0f, 0.0f, 9.81f};

ESKF::ESKF() {
    sigma_gyro = 0.005f;
    sigma_accel = 0.1f;
    sigma_gyro_bias = 0.0001f;
    sigma_accel_bias = 0.01f;
    sigma_accel_meas = 0.5f;
    sigma_baro_meas = 1.0f;
    sigma_gps_pos = 5.0f;
    sigma_gps_vel = 0.5f;
    boost_accel_threshold = 12.0f;
    outlier_threshold_sigmas = 3.0f;

    q = Quat::identity();
    p[0] = p[1] = p[2] = 0;
    v[0] = v[1] = v[2] = 0;
    bg[0] = bg[1] = bg[2] = 0;
    ba[0] = ba[1] = ba[2] = 0;

    memset(P, 0, sizeof(P));
    P[0*N+0] = P[1*N+1] = P[2*N+2] = 0.1f;
    P[3*N+3] = P[4*N+4] = P[5*N+5] = 100.0f;
    P[6*N+6] = P[7*N+7] = P[8*N+8] = 100.0f;
    P[9*N+9]  = P[10*N+10] = P[11*N+11] = 0.01f;
    P[12*N+12] = P[13*N+13] = P[14*N+14] = 0.25f;

    last_innovation_mag = 0;
    correction_count = 0;
    convergence_count = 0;
    reference_altitude = 0;
}

void ESKF::init(float ax, float ay, float az, float baro_altitude) {
    float grav_mag = sqrtf(ax*ax + ay*ay + az*az);
    if (grav_mag > 0.1f) {
        float gb[3] = {ax/grav_mag, ay/grav_mag, az/grav_mag};
        float gw[3] = {0, 0, 1};
        q = Quat::fromTwoVectors(gb, gw);
    } else q = Quat::identity();

    reference_altitude = baro_altitude;
    p[0] = p[1] = p[2] = 0;
    v[0] = v[1] = v[2] = 0;
    bg[0] = bg[1] = bg[2] = 0;
    ba[0] = ba[1] = ba[2] = 0;

    memset(P, 0, sizeof(P));
    P[0*N+0] = P[1*N+1] = P[2*N+2] = 0.1f;
    P[3*N+3] = P[4*N+4] = P[5*N+5] = 100.0f;
    P[6*N+6] = P[7*N+7] = P[8*N+8] = 100.0f;
    P[9*N+9]  = P[10*N+10] = P[11*N+11] = 0.01f;
    P[12*N+12] = P[13*N+13] = P[14*N+14] = 0.25f;

    last_innovation_mag = 0;
    correction_count = 0;
    convergence_count = 0;
}

void ESKF::predict(const float gyro[3], const float accel[3], float dt) {
    float gc[3] = {gyro[0]-bg[0], gyro[1]-bg[1], gyro[2]-bg[2]};
    float ac[3] = {accel[0]-ba[0], accel[1]-ba[1], accel[2]-ba[2]};

    // Quaternion integration
    float ha[3] = {gc[0]*dt*0.5f, gc[1]*dt*0.5f, gc[2]*dt*0.5f};
    q = (q * Quat::fromAxisAngle(ha)).normalized();

    // Velocity integration
    float aw[3]; q.rotateVector(ac, aw);
    aw[2] += GRAVITY_NED[2];
    v[0] += aw[0]*dt; v[1] += aw[1]*dt; v[2] += aw[2]*dt;

    // Position integration
    p[0] += v[0]*dt; p[1] += v[1]*dt; p[2] += v[2]*dt;

    // Covariance propagation
    float F[225]; computeF(gc, ac, dt, F);
    float Q[225]; computeQ(dt, Q);

    float FP[225]; memset(FP, 0, sizeof(FP));
    for (int i=0;i<N;i++) for (int j=0;j<N;j++) for (int k=0;k<N;k++) FP[i*N+j] += F[i*N+k] * P[k*N+j];

    float FPF[225]; memset(FPF, 0, sizeof(FPF));
    for (int i=0;i<N;i++) for (int j=0;j<N;j++) for (int k=0;k<N;k++) FPF[i*N+j] += FP[i*N+k] * F[j*N+k];

    for (int i=0;i<N*N;i++) P[i] = FPF[i] + Q[i];
}

void ESKF::computeF(const float gc[3], const float ac[3], float dt, float F[225]) {
    memset(F, 0, sizeof(float)*225);
    for (int i=0;i<N;i++) F[i*N+i] = 1.0f;

    float w_sk[9]; skew3(gc, w_sk);
    float a_sk[9]; skew3(ac, a_sk);
    float R[9]; q.toRotationMatrix(R);

    float Ra[9]; memset(Ra, 0, sizeof(Ra));
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k=0;k<3;k++) Ra[i*3+j] += R[i*3+k] * a_sk[k*3+j];

    for (int i=0;i<3;i++) for (int j=0;j<3;j++) F[i*N+j] = (i==j?1:0) - w_sk[i*3+j]*dt;
    F[0*N+9] = -dt; F[1*N+10] = -dt; F[2*N+11] = -dt;
    F[3*N+6] = dt; F[4*N+7] = dt; F[5*N+8] = dt;
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) F[(6+i)*N+j] = -Ra[i*3+j]*dt;
    F[6*N+12] = -dt; F[7*N+13] = -dt; F[8*N+14] = -dt;
}

void ESKF::computeQ(float dt, float Q[225]) {
    memset(Q, 0, sizeof(float)*225);
    float Qg = sigma_gyro*sigma_gyro;
    float Qa = sigma_accel*sigma_accel;
    float Qbg = sigma_gyro_bias*sigma_gyro_bias;
    float Qba = sigma_accel_bias*sigma_accel_bias;
    Q[0*N+0] = Qg*dt; Q[1*N+1] = Qg*dt; Q[2*N+2] = Qg*dt;
    Q[6*N+6] = Qa*dt; Q[7*N+7] = Qa*dt; Q[8*N+8] = Qa*dt;
    Q[9*N+9] = Qbg*dt; Q[10*N+10] = Qbg*dt; Q[11*N+11] = Qbg*dt;
    Q[12*N+12] = Qba*dt; Q[13*N+13] = Qba*dt; Q[14*N+14] = Qba*dt;
}

void ESKF::correctAccel(const float accel[3]) {
    float mag = v3_norm(accel);
    float noise = sigma_accel_meas;
    if (fabsf(mag - 9.81f) > 2.0f) noise *= (1.0f + fabsf(mag - 9.81f) / 2.0f);

    float gw[3] = {0,0,9.81f};
    float gb[3]; q.rotateVectorInverse(gw, gb);
    float z[3] = {accel[0]-gb[0], accel[1]-gb[1], accel[2]-gb[2]};

    float gsk[9]; skew3(gw, gsk);
    float RT[9]; q.toRotationMatrix(RT);
    for (int i=0;i<3;i++) for (int j=i+1;j<3;j++) { float t=RT[i*3+j]; RT[i*3+j]=RT[j*3+i]; RT[j*3+i]=t; }

    float Ha[9]; memset(Ha,0,sizeof(Ha));
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) for (int k=0;k<3;k++) Ha[i*3+j] += -RT[i*3+k] * gsk[k*3+j];

    float H[3*15]; memset(H,0,sizeof(H));
    for (int i=0;i<3;i++) for (int j=0;j<3;j++) H[i*N+j] = Ha[i*3+j];

    float Rm[9]; memset(Rm,0,sizeof(Rm));
    float n2 = noise*noise; Rm[0]=n2; Rm[4]=n2; Rm[8]=n2;

    if (kalmanUpdate(H, z, Rm, 3)) { correction_count++; if (correction_count>100) convergence_count++; }
}

void ESKF::correctBaro(float altitude) {
    float est = reference_altitude - p[2];
    float z = altitude - est;
    float H[15]; memset(H,0,sizeof(H)); H[5] = -1.0f;
    float R = sigma_baro_meas * sigma_baro_meas;
    if (kalmanUpdate(H, &z, &R, 1)) correction_count++;
}

void ESKF::correctGPS(const float pos[3], const float vel[3], float pn, float vn) {
    float z[6] = {pos[0]-p[0], pos[1]-p[1], pos[2]-p[2], vel[0]-v[0], vel[1]-v[1], vel[2]-v[2]};
    float H[6*15]; memset(H,0,sizeof(H));
    H[0*N+3]=1; H[1*N+4]=1; H[2*N+5]=1; H[3*N+6]=1; H[4*N+7]=1; H[5*N+8]=1;
    float R[36]; memset(R,0,sizeof(R));
    R[0]=pn*pn; R[7]=pn*pn; R[14]=pn*pn*2; R[21]=vn*vn; R[28]=vn*vn; R[35]=vn*vn;
    if (kalmanUpdate(H, z, R, 6)) correction_count++;
}

bool ESKF::kalmanUpdate(const float* H, const float* z, const float* R, int m) {
    // S = H*P*H^T + R
    float S[36]; memset(S,0,sizeof(S));
    float HP[15*15]; memset(HP,0,sizeof(float)*m*N);
    for (int i=0;i<m;i++) for (int j=0;j<N;j++) for (int k=0;k<N;k++) HP[i*N+j] += H[i*N+k] * P[k*N+j];
    for (int i=0;i<m;i++) for (int j=0;j<m;j++) for (int k=0;k<N;k++) S[i*m+j] += HP[i*N+k] * H[j*N+k];
    for (int i=0;i<m*m;i++) S[i] += R[i];

    if (isOutlier(H, z, R, m)) return false;

    // Invert S
    float Si[36]; memset(Si,0,sizeof(Si));
    for (int i=0;i<m;i++) Si[i*m+i] = 1;
    float Sc[36]; memcpy(Sc, S, sizeof(float)*m*m);
    for (int c=0;c<m;c++) {
        int pv=c; float mv=fabsf(Sc[c*m+c]);
        for (int r=c+1;r<m;r++) { float v=fabsf(Sc[r*m+c]); if(v>mv){mv=v;pv=r;} }
        if (pv!=c) for (int j=0;j<m;j++) { float t=Sc[c*m+j]; Sc[c*m+j]=Sc[pv*m+j]; Sc[pv*m+j]=t; t=Si[c*m+j]; Si[c*m+j]=Si[pv*m+j]; Si[pv*m+j]=t; }
        float pv2=Sc[c*m+c]; if (fabsf(pv2)<1e-15f) continue;
        float ip=1.0f/pv2;
        for (int j=0;j<m;j++) { Sc[c*m+j]*=ip; Si[c*m+j]*=ip; }
        for (int r=0;r<m;r++) { if(r==c) continue; float f=Sc[r*m+c]; for(int j=0;j<m;j++){Sc[r*m+j]-=f*Sc[c*m+j];Si[r*m+j]-=f*Si[c*m+j];} }
    }

    // K = P * H^T * Si
    float PHt[15*6]; memset(PHt,0,sizeof(float)*N*m);
    for (int i=0;i<N;i++) for (int j=0;j<m;j++) for (int k=0;k<N;k++) PHt[i*m+j] += P[i*N+k] * H[j*N+k];
    float K[15*6]; memset(K,0,sizeof(float)*N*m);
    for (int i=0;i<N;i++) for (int j=0;j<m;j++) for (int k=0;k<m;k++) K[i*m+j] += PHt[i*m+k] * Si[k*m+j];

    // δx = K * z
    float dx[15]; memset(dx,0,sizeof(dx));
    for (int i=0;i<N;i++) for (int j=0;j<m;j++) dx[i] += K[i*m+j] * z[j];

    last_innovation_mag = 0;
    for (int i=0;i<m;i++) last_innovation_mag += z[i]*z[i];

    injectErrorState(dx);

    // P = (I - K*H) * P
    float KH[225]; memset(KH,0,sizeof(KH));
    for (int i=0;i<N;i++) for (int j=0;j<N;j++) for (int k=0;k<m;k++) KH[i*N+j] += K[i*m+k] * H[k*N+j];
    float IKH[225]; for (int i=0;i<N;i++) for (int j=0;j<N;j++) IKH[i*N+j] = (i==j?1:0) - KH[i*N+j];
    float Pn[225]; memset(Pn,0,sizeof(Pn));
    for (int i=0;i<N;i++) for (int j=0;j<N;j++) for (int k=0;k<N;k++) Pn[i*N+j] += IKH[i*N+k] * P[k*N+j];
    memcpy(P, Pn, sizeof(P));
    return true;
}

bool ESKF::isOutlier(const float* H, const float* z, const float* R, int m) {
    float S[36]; memset(S,0,sizeof(S));
    float HP[15*15]; memset(HP,0,sizeof(float)*m*N);
    for (int i=0;i<m;i++) for (int j=0;j<N;j++) for (int k=0;k<N;k++) HP[i*N+j] += H[i*N+k] * P[k*N+j];
    for (int i=0;i<m;i++) for (int j=0;j<m;j++) for (int k=0;k<N;k++) S[i*m+j] += HP[i*N+k] * H[j*N+k];
    for (int i=0;i<m*m;i++) S[i] += R[i];

    float Si[36]; memset(Si,0,sizeof(Si));
    for (int i=0;i<m;i++) Si[i*m+i] = 1;
    float Sc[36]; memcpy(Sc, S, sizeof(float)*m*m);
    for (int c=0;c<m;c++) {
        int pv=c; float mv=fabsf(Sc[c*m+c]);
        for (int r=c+1;r<m;r++) { float v=fabsf(Sc[r*m+c]); if(v>mv){mv=v;pv=r;} }
        if (pv!=c) for (int j=0;j<m;j++) { float t=Sc[c*m+j]; Sc[c*m+j]=Sc[pv*m+j]; Sc[pv*m+j]=t; t=Si[c*m+j]; Si[c*m+j]=Si[pv*m+j]; Si[pv*m+j]=t; }
        float pv2=Sc[c*m+c]; if (fabsf(pv2)<1e-15f) continue;
        float ip=1.0f/pv2;
        for (int j=0;j<m;j++) { Sc[c*m+j]*=ip; Si[c*m+j]*=ip; }
        for (int r=0;r<m;r++) { if(r==c) continue; float f=Sc[r*m+c]; for(int j=0;j<m;j++){Sc[r*m+j]-=f*Sc[c*m+j];Si[r*m+j]-=f*Si[c*m+j];} }
    }

    float d2 = 0;
    for (int i=0;i<m;i++) for (int j=0;j<m;j++) d2 += z[i] * Si[i*m+j] * z[j];
    return d2 > outlier_threshold_sigmas*outlier_threshold_sigmas*m;
}

void ESKF::injectErrorState(const float dx[15]) {
    float dt[3] = {dx[0],dx[1],dx[2]};
    q = Quat::smallAngleUpdate(q, dt);
    p[0] += dx[3]; p[1] += dx[4]; p[2] += dx[5];
    v[0] += dx[6]; v[1] += dx[7]; v[2] += dx[8];
    bg[0] += dx[9]; bg[1] += dx[10]; bg[2] += dx[11];
    ba[0] += dx[12]; ba[1] += dx[13]; ba[2] += dx[14];
}

void ESKF::getFusedState(FusedState& out, uint32_t timestamp_us) {
    out.quat_w = q.w; out.quat_x = q.x; out.quat_y = q.y; out.quat_z = q.z;
    out.pos_n = p[0]; out.pos_e = p[1]; out.pos_d = p[2];
    out.vel_n = v[0]; out.vel_e = v[1]; out.vel_d = v[2];
    out.altitude = reference_altitude - p[2];
    out.gyro_bias_x = bg[0]; out.gyro_bias_y = bg[1]; out.gyro_bias_z = bg[2];
    out.accel_bias_x = ba[0]; out.accel_bias_y = ba[1]; out.accel_bias_z = ba[2];
    out.corrected_accel[0] = 0; out.corrected_accel[1] = 0; out.corrected_accel[2] = 0;
    out.corrected_gyro[0] = 0; out.corrected_gyro[1] = 0; out.corrected_gyro[2] = 0;
    out.corrected_pressure = 0;

    float pv = P[3*N+3] + P[4*N+4] + P[5*N+5];
    float av = P[0*N+0] + P[1*N+1] + P[2*N+2];
    out.pos_uncertainty = sqrtf(pv/3.0f);
    out.att_uncertainty = sqrtf(av/3.0f) * 57.2958f;
    float pc = 1.0f/(1.0f+out.pos_uncertainty);
    float ac = 1.0f/(1.0f+out.att_uncertainty*0.1f);
    out.confidence = (pc+ac)*0.5f;
    if (out.confidence > 1.0f) out.confidence = 1.0f;
    out.sensor_status = 0;
    out.timestamp_us = timestamp_us;
}

// =============================================================================
// [4] FRAME FORMATS (APID 0 and APID 1 — identical to RapidFish v2)
// =============================================================================

struct __attribute__((packed)) LogFrameCore {
    uint32_t sync_word;
    uint32_t timestamp;
    uint8_t  apid;
    uint8_t  flight_state;
    uint8_t  flash_used;
    int8_t   core_temp;
    int16_t  ax, ay, az;
    int16_t  gx, gy, gz;
    uint32_t pressure;
    int8_t   temperature;
    uint8_t  bat_voltage;
    uint8_t  p1_voltage;
    uint8_t  p2_voltage;
};
static_assert(sizeof(LogFrameCore) == 32, "LogFrameCore must be 32 bytes");

struct __attribute__((packed)) LogFrameGPS {
    uint32_t sync_word;
    uint32_t timestamp;
    uint8_t  apid;
    int32_t  lat;
    int32_t  lon;
    uint16_t gps_alt;
    uint8_t  state;
    uint8_t  sats;
    uint32_t gps_time;
    uint8_t  hdop;
    int16_t  mx, my, mz;
};
static_assert(sizeof(LogFrameGPS) == 32, "LogFrameGPS must be 32 bytes");

union LogFrame {
    LogFrameCore core;
    LogFrameGPS  gps;
    uint8_t      bytes[32];
};

// =============================================================================
// [5] HARDWARE OBJECTS & STATE
// =============================================================================

Adafruit_LSM6DSO32 lsm_dsox;
SparkFun_LSM6DSV16X_SPI lsm_dsv;
bool use_dsox = false;
Adafruit_BMP3XX bmp;

#define RADIO_SPI_CLOCK_HZ 8000000
LR2021 radio = new Module(RADIO_CS_PIN, RADIO_IRQ_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN, SPI1,
                          SPISettings(RADIO_SPI_CLOCK_HZ, MSBFIRST, SPI_MODE0));
bool radio_ready = false;

static uint8_t radio_tx_buf[32];
static size_t  radio_tx_len = 0;
volatile bool  radio_tx_busy = false;
volatile bool  radio_tx_done = false;
static volatile uint32_t radio_tx_busy_start_ms = 0;
static volatile uint64_t radio_tx_busy_start_us = 0;
#define RADIO_TX_BUSY_TIMEOUT_MS 500
#define RADIO_TX_BUSY_TIMEOUT_US (RADIO_TX_BUSY_TIMEOUT_MS * 1000u)

uint32_t radio_tx_attempted = 0;
uint32_t radio_tx_dropped   = 0;
uint32_t last_radio_tx = 0;
uint32_t last_gps_tx   = 0;

TinyGPSPlus tinyGPS;
bool gps_ready = false;
uint32_t gps_last_nmea_ms = 0;
uint32_t gps_last_valid_fix_ms = 0;

ESKF eskf;
critical_section_t eskf_lock;
volatile bool fused_state_ready = false;
ESKF::FusedState shared_fused_state;

critical_section_t gps_mailbox_lock;
volatile bool gps_pending_flag = false;
struct GpsMail { float pos_n,pos_e,pos_d,vel_n,vel_e,vel_d,pos_noise,vel_noise; bool valid; } gps_mail;

volatile uint8_t system_errors = 0;
critical_section_t system_errors_lock;
volatile bool core1_init_complete = false;
volatile int radio_error_code = 0;

// =============================================================================
// [6] FORWARD DECLARATIONS
// =============================================================================

void radioInit();
bool radioTransmitFrame();
bool radioTransmitGpsFrame();
static bool radioTransmit(const uint8_t* frame, size_t len);
void radioTxDoneISR();
void gpsInit();
void handleGps();
void buildCoreFrame(LogFrameCore* f, const ESKF::FusedState& state, uint32_t now);
void buildGpsFrame(LogFrameGPS* f, const ESKF::FusedState& state, uint32_t now);

// =============================================================================
// [7] CORE 0: SETUP & TELEMETRY
// =============================================================================

void setup() {
    Serial.begin(115200);
    critical_section_init(&system_errors_lock);
    critical_section_init(&eskf_lock);
    critical_section_init(&gps_mailbox_lock);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    analogReadResolution(12);

    uint32_t bt = millis();
    while (!Serial && millis()-bt < 5000) delay(10);

    Serial.println("\n==============================================");
    Serial.println("   INS Test Firmware — ESKF Sensor Fusion");
    Serial.println("==============================================");

    radioInit();
    gpsInit();

    Serial.println("Waiting for core 1 (sensor fusion) to initialize...");
    while (!core1_init_complete) delay(10);

    Serial.println("System ready. Transmitting fused state via APID 0/1.");
    Serial.println("Commands: STATUS, RESET, HELP\n");
}

void loop() {
    handleGps();
    uint32_t now = millis();

    // Radio TX watchdog
    if (radio_tx_busy) {
        bool clr = false;
        if (radio_tx_done) { radio.finishTransmit(); clr = true; }
        else if (time_us_64() - radio_tx_busy_start_us >= RADIO_TX_BUSY_TIMEOUT_US) { radio.finishTransmit(); clr = true; }
        if (clr) { radio_tx_busy = false; radio_tx_done = false; radio_tx_busy_start_ms = 0; radio_tx_busy_start_us = 0; }
    }

    // APID 0 at 80 ms
    if (now - last_radio_tx >= RADIO_TX_INTERVAL_MS) { if (radioTransmitFrame()) last_radio_tx = now; }
    // APID 1 at 1 Hz
    if (now - last_gps_tx >= 1000) { if (radioTransmitGpsFrame()) last_gps_tx = now; }

    // Serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n'); cmd.trim();
        if (cmd == "HELP") {
            Serial.println("STATUS — print fused state");
            Serial.println("RESET  — reset filter");
            Serial.println("HELP   — this");
        } else if (cmd == "STATUS") {
            critical_section_enter_blocking(&eskf_lock);
            if (fused_state_ready) {
                ESKF::FusedState s = shared_fused_state;
                critical_section_exit(&eskf_lock);
                Serial.printf("Att: w=%.4f x=%.4f y=%.4f z=%.4f\n", s.quat_w,s.quat_x,s.quat_y,s.quat_z);
                Serial.printf("Pos: N=%.1f E=%.1f D=%.1f m\n", s.pos_n,s.pos_e,s.pos_d);
                Serial.printf("Vel: N=%.2f E=%.2f D=%.2f m/s\n", s.vel_n,s.vel_e,s.vel_d);
                Serial.printf("Alt: %.1f m  Conf: %.2f  PosU: %.1f m  AttU: %.1f°\n", s.altitude,s.confidence,s.pos_uncertainty,s.att_uncertainty);
                Serial.printf("GB: %.4f %.4f %.4f  AB: %.4f %.4f %.4f\n", s.gyro_bias_x,s.gyro_bias_y,s.gyro_bias_z,s.accel_bias_x,s.accel_bias_y,s.accel_bias_z);
            } else { critical_section_exit(&eskf_lock); Serial.println("Not ready."); }
        } else if (cmd == "RESET") {
            critical_section_enter_blocking(&eskf_lock);
            fused_state_ready = false;
            critical_section_exit(&eskf_lock);
            Serial.println("Reset requested.");
        }
    }
}

// =============================================================================
// [8] CORE 1: SENSOR FUSION ENGINE
// =============================================================================

void setup1() {
    delay(5000);

    pinMode(LSM_CS_PIN, OUTPUT);
    digitalWrite(LSM_CS_PIN, HIGH); delay(10);

    SPI.setRX(16); SPI.setSCK(18); SPI.setTX(19); SPI.begin();

    digitalWrite(LSM_CS_PIN, LOW); delay(5);
    digitalWrite(LSM_CS_PIN, HIGH); delay(5);

    if (lsm_dsox.begin_SPI(LSM_CS_PIN)) {
        use_dsox = true;
        lsm_dsox.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
        lsm_dsox.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
        lsm_dsox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
        Serial.println("Core1: LSM6DSO32");
    } else {
        digitalWrite(LSM_CS_PIN, HIGH); delay(10);
        if (lsm_dsv.begin(LSM_CS_PIN)) {
            use_dsox = false;
            lsm_dsv.deviceReset(); while (!lsm_dsv.getDeviceReset()) delay(1);
            lsm_dsv.enableBlockDataUpdate();
            lsm_dsv.setAccelDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setAccelFullScale(LSM6DSV16X_16g);
            lsm_dsv.setGyroDataRate(LSM6DSV16X_ODR_AT_1920Hz);
            lsm_dsv.setGyroFullScale(LSM6DSV16X_2000dps);
            Serial.println("Core1: LSM6DSV16X");
        } else {
            critical_section_enter_blocking(&system_errors_lock);
            system_errors |= 1;
            critical_section_exit(&system_errors_lock);
            Serial.println("Core1: IMU FAILED");
        }
    }

    Wire.setSDA(BMP_SDA_PIN); Wire.setSCL(BMP_SCL_PIN); Wire.begin(); Wire.setClock(400000);
    bool bmp_ok = false;
    for (int a=0; a<3 && !bmp_ok; a++) { if (a>0) {delay(50);Wire.end();Wire.begin();Wire.setClock(400000);} bmp_ok = bmp.begin_I2C(0x76, &Wire); }
    if (!bmp_ok) {
        critical_section_enter_blocking(&system_errors_lock);
        system_errors |= 2;
        critical_section_exit(&system_errors_lock);
        Serial.println("Core1: Baro FAILED");
    } else {
        bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
        bmp.setOutputDataRate(BMP3_ODR_200_HZ);
        Serial.println("Core1: BMP390");
    }

    float iax=0, iay=0, iaz=0, ialt=0;
    if (!(system_errors & 1)) {
        if (use_dsox) { sensors_event_t a,g,t; lsm_dsox.getEvent(&a,&g,&t); iax=a.acceleration.x; iay=a.acceleration.y; iaz=a.acceleration.z; }
        else { sfe_lsm_data_t ad; if(lsm_dsv.checkStatus()){lsm_dsv.getAccel(&ad);iax=(ad.xData/1000.0f)*9.81f;iay=(ad.yData/1000.0f)*9.81f;iaz=(ad.zData/1000.0f)*9.81f;} }
    }
    if (!(system_errors & 2)) { bmp.performReading(); ialt = bmp.readAltitude(1013.25f); }

    eskf.init(iax, iay, iaz, ialt);
    Serial.println("Core1: ESKF initialized");
    core1_init_complete = true;
}

void loop1() {
    static uint32_t last_us = 0;
    static uint8_t dec = 0;
    static uint32_t ct = 0;
    static float balt = 0, bpress = 101325, rph = 1013.25f;

    if (system_errors > 0) return;

    if (micros() - last_us >= 1000) {
        if (micros() - last_us > 10000) last_us = micros(); else last_us += 1000;

        float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        if (!(system_errors & 1)) {
            if (use_dsox) {
                sensors_event_t a,g,t; lsm_dsox.getEvent(&a,&g,&t);
                ax=a.acceleration.x; ay=a.acceleration.y; az=a.acceleration.z;
                gx=g.gyro.x; gy=g.gyro.y; gz=g.gyro.z;
            } else {
                sfe_lsm_data_t ad, gd;
                if (lsm_dsv.checkStatus()) {
                    lsm_dsv.getAccel(&ad); lsm_dsv.getGyro(&gd);
                    ax=(ad.xData/1000.0f)*9.81f; ay=(ad.yData/1000.0f)*9.81f; az=(ad.zData/1000.0f)*9.81f;
                    gx=(gd.xData/1000.0f)*0.0174533f; gy=(gd.yData/1000.0f)*0.0174533f; gz=(gd.zData/1000.0f)*0.0174533f;
                }
            }
        }

        float gyro[3] = {gx,gy,gz}, accel[3] = {ax,ay,az};
        eskf.predict(gyro, accel, 0.001f);

        // Baro at 200 Hz
        if (++dec >= 5) { dec = 0;
            if (!(system_errors & 2)) {
                bmp.performReading(); bpress = bmp.pressure;
                rph = rph*0.9f + (bmp.pressure/100.0f)*0.1f;
                balt = bmp.readAltitude(rph);
            }
            eskf.correctBaro(balt);
        }

        // Accel correction at 50 Hz
        if (micros() - ct >= 20000) { ct = micros(); eskf.correctAccel(accel); }

        // GPS correction
        critical_section_enter_blocking(&gps_mailbox_lock);
        if (gps_pending_flag && gps_mail.valid) {
            float pos[3]={gps_mail.pos_n,gps_mail.pos_e,gps_mail.pos_d};
            float vel[3]={gps_mail.vel_n,gps_mail.vel_e,gps_mail.vel_d};
            float pn=gps_mail.pos_noise, vn=gps_mail.vel_noise;
            gps_pending_flag = false;
            critical_section_exit(&gps_mailbox_lock);
            eskf.correctGPS(pos, vel, pn, vn);
        } else critical_section_exit(&gps_mailbox_lock);

        // Write fused state
        ESKF::FusedState st;
        eskf.getFusedState(st, time_us_64());
        st.corrected_accel[0] = ax - st.accel_bias_x;
        st.corrected_accel[1] = ay - st.accel_bias_y;
        st.corrected_accel[2] = az - st.accel_bias_z;
        st.corrected_gyro[0] = gx - st.gyro_bias_x;
        st.corrected_gyro[1] = gy - st.gyro_bias_y;
        st.corrected_gyro[2] = gz - st.gyro_bias_z;
        st.corrected_pressure = bpress;
        st.sensor_status = ((system_errors&1)?0:1) | ((system_errors&2)?0:2) | (gps_ready?4:0);

        critical_section_enter_blocking(&eskf_lock);
        shared_fused_state = st;
        fused_state_ready = true;
        critical_section_exit(&eskf_lock);
    }
}

// =============================================================================
// [9] RADIO FUNCTIONS
// =============================================================================

void radioInit() {
    SPI1.setRX(12); SPI1.setSCK(14); SPI1.setTX(15); SPI1.begin();
    int st = radio.begin();
    if (st == RADIOLIB_ERR_NONE) {
        st = radio.setFrequency(RADIO_FREQUENCY_MHZ);
        if (st == RADIOLIB_ERR_NONE) {
            radio.setOutputPower(RADIO_POWER_DBM);
            radio.setBandwidth(250.0f); radio.setSpreadingFactor(7);
            radio.setCodingRate(5); radio.setPreambleLength(8);
            radio.setSyncWord(0x12); radio.setCRC(true);
            radio.implicitHeader(28); radio.setDio5Action(radioTxDoneISR);
            radio_ready = true;
            Serial.println("Radio: OK");
        }
    } else {
        radio_error_code = st;
        critical_section_enter_blocking(&system_errors_lock);
        system_errors |= 4;
        critical_section_exit(&system_errors_lock);
        Serial.printf("Radio: fail %d\n", st);
    }
}

void radioTxDoneISR() { radio_tx_done = true; }

static bool radioTransmit(const uint8_t* frame, size_t len) {
    if (!radio_ready || radio_tx_busy) { if(radio_tx_busy) radio_tx_dropped++; return false; }
    radio_tx_attempted++;
    memcpy(radio_tx_buf, frame, len); radio_tx_len = len;
    radio_tx_busy = true; radio_tx_done = false;
    radio_tx_busy_start_ms = millis(); radio_tx_busy_start_us = time_us_64();
    return radio.startTransmit(radio_tx_buf, len) == RADIOLIB_ERR_NONE;
}

bool radioTransmitFrame() {
    critical_section_enter_blocking(&eskf_lock);
    if (!fused_state_ready) { critical_section_exit(&eskf_lock); return false; }
    ESKF::FusedState s = shared_fused_state;
    critical_section_exit(&eskf_lock);
    LogFrameCore f; buildCoreFrame(&f, s, millis());
    return radioTransmit(f.bytes, sizeof(LogFrameCore));
}

bool radioTransmitGpsFrame() {
    critical_section_enter_blocking(&eskf_lock);
    if (!fused_state_ready) { critical_section_exit(&eskf_lock); return false; }
    ESKF::FusedState s = shared_fused_state;
    critical_section_exit(&eskf_lock);
    LogFrameGPS f; buildGpsFrame(&f, s, millis());
    return radioTransmit(f.bytes, sizeof(LogFrameGPS));
}

void buildCoreFrame(LogFrameCore* f, const ESKF::FusedState& s, uint32_t now) {
    f->sync_word = SYNC_WORD; f->timestamp = now; f->apid = 0;
    f->flight_state = 3; f->flash_used = 0;
    f->core_temp = (int8_t)analogReadTemp();
    f->ax = (int16_t)(s.corrected_accel[0] * 100);
    f->ay = (int16_t)(s.corrected_accel[1] * 100);
    f->az = (int16_t)(s.corrected_accel[2] * 100);
    f->gx = (int16_t)(s.corrected_gyro[0] * 1000);
    f->gy = (int16_t)(s.corrected_gyro[1] * 1000);
    f->gz = (int16_t)(s.corrected_gyro[2] * 1000);
    f->pressure = (uint32_t)s.corrected_pressure;
    f->temperature = 0;
    f->bat_voltage = (uint8_t)(analogRead(BAT_ADC_PIN) >> 4);
    f->p1_voltage = 0; f->p2_voltage = 0;
}

void buildGpsFrame(LogFrameGPS* f, const ESKF::FusedState& s, uint32_t now) {
    f->sync_word = SYNC_WORD; f->timestamp = now; f->apid = 1;
    static float olat=0, olon=0;
    static bool oset = false;
    if (!oset && tinyGPS.location.isValid()) { olat=tinyGPS.location.lat(); olon=tinyGPS.location.lng(); oset=true; }
    float lat = olat + s.pos_n / 111320.0f;
    float lon = olon + s.pos_e / (111320.0f * cosf(olat * 0.0174533f));
    f->lat = (int32_t)(lat * 1e7); f->lon = (int32_t)(lon * 1e7);
    f->gps_alt = (uint16_t)(s.altitude * 2.0f);
    f->state = tinyGPS.location.isValid() ? 1 : 0;
    f->sats = (uint8_t)tinyGPS.satellites.value();
    f->gps_time = tinyGPS.time.isValid() ? tinyGPS.date.age() : 0;
    f->hdop = (uint8_t)(tinyGPS.hdop.value() * 10);
    f->mx = f->my = f->mz = 0;
}

// =============================================================================
// [10] GPS FUNCTIONS
// =============================================================================

void gpsInit() {
    Serial1.setTX(GPS_TX_PIN); Serial1.setRX(GPS_RX_PIN); Serial1.begin(GPS_BAUD);
    gps_ready = true;
    Serial.println("GPS: OK");
}

void handleGps() {
    while (Serial1.available()) { char c = Serial1.read(); tinyGPS.encode(c); }
    if (tinyGPS.location.isValid() && tinyGPS.location.age() < 1000) {
        static uint32_t last = 0;
        uint32_t now = millis();
        if (now - last > 200) {
            last = now;
            static float olat=0, olon=0;
            static bool oset=false;
            if (!oset) { olat=tinyGPS.location.lat(); olon=tinyGPS.location.lng(); oset=true; }
            float dlat = tinyGPS.location.lat() - olat;
            float dlon = tinyGPS.location.lng() - olon;
            float pn = dlat * 111320.0f;
            float pe = dlon * 111320.0f * cosf(olat * 0.0174533f);
            float vn = tinyGPS.speed.mps() * cosf(tinyGPS.course.deg() * 0.0174533f);
            float ve = tinyGPS.speed.mps() * sinf(tinyGPS.course.deg() * 0.0174533f);
            float hd = tinyGPS.hdop.value();
            critical_section_enter_blocking(&gps_mailbox_lock);
            gps_mail.pos_n=pn; gps_mail.pos_e=pe; gps_mail.pos_d=0;
            gps_mail.vel_n=vn; gps_mail.vel_e=ve; gps_mail.vel_d=0;
            gps_mail.pos_noise=5.0f*(hd>1?hd:1); gps_mail.vel_noise=0.5f;
            gps_mail.valid = true; gps_pending_flag = true;
            critical_section_exit(&gps_mailbox_lock);
        }
    }
}