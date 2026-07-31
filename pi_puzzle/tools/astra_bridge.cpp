/*
 * astra_bridge - Orbbec Astra S 取帧桥接程序
 *
 * 基于 OpenNI2 C++ SDK 采集 RGB + Depth 帧, 通过 stdin/stdout
 * 二进制协议与 Python 端 (camera_backend.py) 通信。
 *
 * 协议:
 *   命令 (stdin):  'f' = 抓一帧快照, 'q' = 退出
 *   响应 (stdout): 若干数据包后跟结束标记
 *     数据包: 'ASTR' + type(1B) + width(u32LE) + height(u32LE) + size(u32LE) + payload
 *       type='C': payload = w*h*3 字节 RGB888
 *       type='D': payload = w*h*2 字节 uint16 深度(mm)
 *     结束标记: 'AEND'
 *
 * 所有日志输出到 stderr, 保证 stdout 只有协议数据。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <vector>
#include <OpenNI.h>

using namespace openni;

static const int READ_TIMEOUT_MS = 2000;

struct FrameBuffer {
    int width = 0;
    int height = 0;
    std::vector<unsigned char> data;
    bool valid = false;
};

static bool write_all(int fd, const void* buf, size_t n) {
    const unsigned char* p = (const unsigned char*)buf;
    while (n > 0) {
        ssize_t w = ::write(fd, p, n);
        if (w <= 0) return false;
        p += w;
        n -= (size_t)w;
    }
    return true;
}

static bool send_packet(char type, const FrameBuffer& fb) {
    unsigned char header[13];
    header[0] = (unsigned char)type;
    unsigned int w = (unsigned int)fb.width;
    unsigned int h = (unsigned int)fb.height;
    unsigned int sz = (unsigned int)fb.data.size();
    memcpy(header + 1, &w, 4);
    memcpy(header + 5, &h, 4);
    memcpy(header + 9, &sz, 4);

    if (!write_all(STDOUT_FILENO, "ASTR", 4)) return false;
    if (!write_all(STDOUT_FILENO, header, sizeof(header))) return false;
    if (!write_all(STDOUT_FILENO, fb.data.data(), fb.data.size())) return false;
    return true;
}

int main(int /*argc*/, char** /*argv*/)
{
    setvbuf(stderr, NULL, _IONBF, 0);
    // stdout 不做缓冲, 协议数据立即送达 Python
    setvbuf(stdout, NULL, _IONBF, 0);

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        fprintf(stderr, "[Bridge] 初始化失败: %s\n", OpenNI::getExtendedError());
        return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        fprintf(stderr, "[Bridge] 无法打开设备: %s\n", OpenNI::getExtendedError());
        return 2;
    }
    fprintf(stderr, "[Bridge] 设备已打开: %s\n", device.getDeviceInfo().getName());

    VideoStream color, depth;
    bool hasColor = false, hasDepth = false;

    if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
        rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK && color.start() == STATUS_OK) {
            hasColor = true;
            VideoMode vm = color.getVideoMode();
            fprintf(stderr, "[Bridge] RGB 流: %dx%d@%d\n",
                    vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
        } else {
            fprintf(stderr, "[Bridge] RGB 流启动失败: %s\n", OpenNI::getExtendedError());
        }
    }

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc == STATUS_OK && depth.start() == STATUS_OK) {
            hasDepth = true;
            VideoMode vm = depth.getVideoMode();
            fprintf(stderr, "[Bridge] Depth 流: %dx%d@%d\n",
                    vm.getResolutionX(), vm.getResolutionY(), vm.getFps());
        } else {
            fprintf(stderr, "[Bridge] Depth 流启动失败: %s\n", OpenNI::getExtendedError());
        }
    }

    if (!hasColor && !hasDepth) {
        fprintf(stderr, "[Bridge] 没有可用的流\n");
        return 3;
    }

    FrameBuffer colorBuf, depthBuf;
    VideoFrameRef frame;
    bool running = true;

    while (running) {
        // 1. 轮询最新帧到缓冲区
        VideoStream* streams[2];
        int nstreams = 0;
        if (hasColor) streams[nstreams++] = &color;
        if (hasDepth) streams[nstreams++] = &depth;

        int changed = -1;
        rc = OpenNI::waitForAnyStream(streams, nstreams, &changed, 20);
        if (rc == STATUS_OK && changed >= 0) {
            VideoStream* s = streams[changed];
            if (s->readFrame(&frame) == STATUS_OK) {
                int w = frame.getWidth(), h = frame.getHeight();
                int sz = frame.getDataSize();
                if (s == &color &&
                    frame.getVideoMode().getPixelFormat() == PIXEL_FORMAT_RGB888) {
                    colorBuf.width = w;
                    colorBuf.height = h;
                    colorBuf.data.resize((size_t)sz);
                    memcpy(colorBuf.data.data(), frame.getData(), (size_t)sz);
                    colorBuf.valid = true;
                } else if (s == &depth &&
                    (frame.getVideoMode().getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM ||
                     frame.getVideoMode().getPixelFormat() == PIXEL_FORMAT_DEPTH_100_UM)) {
                    depthBuf.width = w;
                    depthBuf.height = h;
                    depthBuf.data.resize((size_t)sz);
                    memcpy(depthBuf.data.data(), frame.getData(), (size_t)sz);
                    depthBuf.valid = true;
                }
            }
        }

        // 2. 非阻塞检查 stdin 命令
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv = {0, 0};
        if (select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) > 0) {
            unsigned char cmd;
            ssize_t n = ::read(STDIN_FILENO, &cmd, 1);
            if (n <= 0) {
                // stdin 关闭, Python 端已退出
                break;
            }
            if (cmd == 'f') {
                bool ok = true;
                if (colorBuf.valid) ok = send_packet('C', colorBuf);
                if (ok && depthBuf.valid) ok = send_packet('D', depthBuf);
                if (ok) ok = write_all(STDOUT_FILENO, "AEND", 4);
                if (!ok) {
                    fprintf(stderr, "[Bridge] 写 stdout 失败 (管道断开?)\n");
                    break;
                }
            } else if (cmd == 'q') {
                running = false;
            }
        }
    }

    if (hasColor) { color.stop(); color.destroy(); }
    if (hasDepth) { depth.stop(); depth.destroy(); }
    device.close();
    OpenNI::shutdown();
    fprintf(stderr, "[Bridge] 已退出\n");
    return 0;
}
