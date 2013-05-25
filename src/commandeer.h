/**
 * Copyright (c) 2012 Martin M Reed
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __FFVIDEOCAMERAAPP_HPP__
#define __FFVIDEOCAMERAAPP_HPP__

#include <QtCore/QObject>
#include <QtCore/QMetaType>

#include <bb/cascades/ForeignWindowControl>
#include <bb/cascades/Button>
#include <bb/cascades/Label>

#include "libffbb/ffbbenc.h"
#include "libffbb/ffbbdec.h"
#include <deque>

#include "cmdr/cmdr_client.h"
#include "cmdr/bps_tracker.h"

using namespace bb::cascades;

void connected_callback(void *cookie);
void frame_index_callback(int index, void* cookie);

void ffd_context_close(ffdec_context *ffd_context, void *arg);
int ffd_read_callback(ffdec_context *ffd_context, uint8_t *buf, ssize_t size, void *arg);
void ffd_frame_callback(ffdec_context *ffd_context, AVFrame *frame, int index, void *arg);

void ffe_context_close(ffenc_context *ffe_context, void *arg);
void vf_callback(camera_handle_t handle, camera_buffer_t* buf, void* arg);
void ffe_write_callback(ffenc_context *ffe_context, uint8_t *buf, ssize_t size, void *arg);
bool ffe_frame_callback(ffenc_context *ffe_context, AVFrame *frame, int index, void *arg);

class CommandeerApp : public QObject
{
    friend void connected_callback(void *cookie);
    friend void frame_index_callback(int index, void* cookie);

    friend void ffd_context_close(ffdec_context *ffd_context, void *arg);
    friend int ffd_read_callback(ffdec_context *ffd_context, uint8_t *buf, ssize_t size, void *arg);
    friend void ffd_frame_callback(ffdec_context *ffd_context, AVFrame *frame, int index, void *arg);

    friend void ffe_context_close(ffenc_context *ffe_context, void *arg);
    friend void vf_callback(camera_handle_t handle, camera_buffer_t* buf, void* arg);
    friend void ffe_write_callback(ffenc_context *ffe_context, uint8_t *buf, ssize_t size, void *arg);
    friend bool ffe_frame_callback(ffenc_context *ffe_context, AVFrame *frame, int index, void *arg);

Q_OBJECT
    public slots:

    void onWindowAttached(screen_window_t win, const QString &group, const QString &id);
    void onStartFront();
    void onStartRear();
    void onStopCamera();
    void onStartStopDecoder();
    void onStopDecoder();
    void onStartStopRecording();

public:

    CommandeerApp();
    ~CommandeerApp();

private:

    int createViewfinder(camera_unit_t cameraUnit, const QString &group, const QString &id);

    void update_fps(std::deque<long> &fps, long buf);
    void print_fps(long timestamp);
    void show_frame(AVFrame *frame);

    bool start_encoder(CodecID codec_id);
    bool start_decoder(CodecID codec_id);

    ForeignWindowControl *viewfinderWindow;
    Button *frontCameraButton;
    Button *rearCameraButton;
    Button *playButton;
    Button *stopCameraButton;
    Button *recordButton;
    camera_handle_t mCameraHandle;
    camera_unit_t mCameraUnit;

    bool record, decode;
    int frame_rate;
    std::deque<long> fps_camera;

    cmdr::cmdr_client *client;
    pthread_mutex_t connected_mutex;
    pthread_cond_t connected_cond;
    bool client_ready;

    ffenc_context *ffe_context;
    std::map<int, long> frame_indices;
    cmdr::bps_tracker* bps_tracker_enc;
    std::deque<long> fps_enc;

    ffdec_context *ffd_context;
    cmdr::bps_tracker* bps_tracker_dec;
    std::deque<long> fps_dec;
};

#endif
