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

#include "commandeer.h"
#include "time/_time.h"

#include <bb/cascades/Application>
#include <bb/cascades/Window>
#include <bb/cascades/ForeignWindowControl>
#include <bb/cascades/Container>
#include <bb/cascades/StackLayout>
#include <bb/cascades/DockLayout>
#include <bb/cascades/Button>
#include <bb/cascades/Label>
#include <bb/cascades/Page>

#include <camera/camera_api.h>
#include <screen/screen.h>

#include <unistd.h>
#include <pthread.h>

using namespace bb::cascades;

#define VIDEO_WIDTH 288
#define VIDEO_HEIGHT 512
#define CODEC_ID CODEC_ID_MPEG2VIDEO

CommandeerApp::CommandeerApp()
        : mCameraHandle(CAMERA_HANDLE_INVALID), record(false), decode(false), client_ready(false)
{
    viewfinderWindow = ForeignWindowControl::create().windowId(QString("cameraViewfinder"));

    // Allow Cascades to update the native window's size, position, and visibility, but not the source-size.
    // Cascades may otherwise attempt to redefine the buffer source-size to match the window size, which would yield
    // undesirable results.  You can experiment with this if you want to see what I mean.
    viewfinderWindow->setUpdatedProperties(WindowProperty::Position | WindowProperty::Size | WindowProperty::Visible);

    QObject::connect(viewfinderWindow,
            SIGNAL(windowAttached(screen_window_t, const QString &, const QString &)),
            this, SLOT(onWindowAttached(screen_window_t, const QString &,const QString &)));

    frontCameraButton = Button::create("Front")
            .onClicked(this, SLOT(onStartFront()));

    rearCameraButton = Button::create("Rear")
            .onClicked(this, SLOT(onStartRear()));

    stopCameraButton = Button::create("Stop Camera")
            .onClicked(this, SLOT(onStopCamera()));
    stopCameraButton->setVisible(false);

    playButton = Button::create("Play")
            .onClicked(this, SLOT(onStartStopDecoder()));

    recordButton = Button::create("Record")
            .onClicked(this, SLOT(onStartStopRecording()));
    recordButton->setVisible(false);

    Container* container = Container::create().layout(DockLayout::create())
            .add(Container::create().horizontal(HorizontalAlignment::Center).vertical(VerticalAlignment::Center).add(viewfinderWindow))
            .add(Container::create().horizontal(HorizontalAlignment::Left).vertical(VerticalAlignment::Top))
            .add(Container::create().horizontal(HorizontalAlignment::Center).vertical(VerticalAlignment::Bottom).layout(StackLayout::create().orientation(LayoutOrientation::LeftToRight))
            .add(frontCameraButton)
            .add(rearCameraButton)
            .add(playButton)
            .add(recordButton)
            .add(stopCameraButton));

    Application::instance()->setScene(Page::create().content(container));

    ffe_context = new ffenc_context();
    ffd_context = new ffdec_context();

//    const char* server = "192.168.1.14"; // portland_brew
    const char* server = "192.168.1.10"; // home
//    const char* server = "10.0.0.7"; // fido
//    const char* server = "10.5.70.139"; // frothy
//    const char* server = "10.1.45.143"; // asurion
//    const char* server = "172.20.133.139"; // bbjam

    client = new cmdr::cmdr_client(server, 1031, "1", "2");
    client->connected_callback = connected_callback;
    client->connected_callback_cookie = this;
    client->frame_index_callback = frame_index_callback;
    client->frame_index_callback_cookie = this;

    bps_tracker_enc = new cmdr::bps_tracker();
    bps_tracker_dec = new cmdr::bps_tracker();

    pthread_mutex_init(&connected_mutex, 0);
    pthread_cond_init(&connected_cond, 0);

    frame_rate = 30;
}

CommandeerApp::~CommandeerApp()
{
    delete viewfinderWindow;

    delete ffe_context;
    delete ffd_context;

    delete bps_tracker_enc;
    delete bps_tracker_dec;

    delete client;

    pthread_mutex_destroy(&connected_mutex);
    pthread_cond_destroy(&connected_cond);
}

void CommandeerApp::onWindowAttached(screen_window_t win, const QString &group, const QString &id)
{
    qDebug() << "onWindowAttached: " << group << ", " << id;

    // set screen properties to mirror if this is the front-facing camera
    int mirror = mCameraUnit == CAMERA_UNIT_FRONT;
    screen_set_window_property_iv(win, SCREEN_PROPERTY_MIRROR, &mirror);

    // put the viewfinder window behind the cascades window
    int z = -1;
    screen_set_window_property_iv(win, SCREEN_PROPERTY_ZORDER, &z);

    // scale the viewfinder window to fit the display
//    int size[] = { 768, 1280 };
//    screen_set_window_property_iv(win, SCREEN_PROPERTY_SIZE, size);

    // seems we still need a workaround in R9 for a potential race due to
    // ForeignWindowControl updating/flushing the window's properties in
    // parallel with the execution of the onWindowAttached() handler.
    viewfinderWindow->setVisible(false);
    viewfinderWindow->setVisible(true);
}

int CommandeerApp::createViewfinder(camera_unit_t cameraUnit, const QString &group, const QString &id)
{
    if (mCameraHandle != CAMERA_HANDLE_INVALID) return EBUSY;

    mCameraUnit = cameraUnit;

    if (camera_open(mCameraUnit, CAMERA_MODE_RW, &mCameraHandle) != CAMERA_EOK) return EIO;

    camera_set_videovf_property(mCameraHandle, CAMERA_IMGPROP_WIN_GROUPID, group.toStdString().c_str());
    camera_set_videovf_property(mCameraHandle, CAMERA_IMGPROP_WIN_ID, id.toStdString().c_str());

    camera_set_videovf_property(mCameraHandle,
            CAMERA_IMGPROP_WIDTH, VIDEO_WIDTH,
            CAMERA_IMGPROP_HEIGHT, VIDEO_HEIGHT);

    camera_set_video_property(mCameraHandle,
            CAMERA_IMGPROP_WIDTH, VIDEO_WIDTH,
            CAMERA_IMGPROP_HEIGHT, VIDEO_HEIGHT);

    if (camera_start_video_viewfinder(mCameraHandle, vf_callback, 0, this) != CAMERA_EOK)
    {
        camera_close(mCameraHandle);
        mCameraHandle = CAMERA_HANDLE_INVALID;
        return EIO;
    }

    frontCameraButton->setVisible(false);
    rearCameraButton->setVisible(false);
    playButton->setVisible(false);
    stopCameraButton->setVisible(true);
    recordButton->setText("Record");
    recordButton->setVisible(true);
    recordButton->setEnabled(true);
    return EOK;
}

void CommandeerApp::onStartFront()
{
    if (!viewfinderWindow) return;
    const QString windowGroup = viewfinderWindow->windowGroup();
    const QString windowId = viewfinderWindow->windowId();
    createViewfinder(CAMERA_UNIT_FRONT, windowGroup, windowId);
}

void CommandeerApp::onStartRear()
{
    if (!viewfinderWindow) return;
    const QString windowGroup = viewfinderWindow->windowGroup();
    const QString windowId = viewfinderWindow->windowId();
    createViewfinder(CAMERA_UNIT_REAR, windowGroup, windowId);
}

void CommandeerApp::onStopCamera()
{
    if (mCameraHandle == CAMERA_HANDLE_INVALID) return;

    // NOTE that closing the camera causes the viewfinder to stop.
    // When the viewfinder stops, it's window is destroyed and the
    // ForeignWindow object will emit a windowDetached signal.
    camera_close(mCameraHandle);
    mCameraHandle = CAMERA_HANDLE_INVALID;

    recordButton->setVisible(false);
    stopCameraButton->setVisible(false);
    frontCameraButton->setVisible(true);
    rearCameraButton->setVisible(true);
    playButton->setVisible(true);
}

void connected_callback(void *cookie)
{
    CommandeerApp* app = (CommandeerApp*) cookie;
    if (app->client_ready) return;
    app->client_ready = true;
    pthread_cond_signal(&app->connected_cond);
}

void CommandeerApp::onStopDecoder()
{
    if (!decode) return;

    decode = false;
    ffd_context->stop();
    playButton->setText("Play");
}

void CommandeerApp::onStartStopDecoder()
{
    if (decode)
    {
        onStopDecoder();
        return;
    }

    client_ready = false;

    if (!client->start())
    {
        fprintf(stderr, "Cannot connect to server\n");
        return;
    }

    if (!start_decoder(CODEC_ID)) return;

    decode = true;

    playButton->setText("Stop");
}

void CommandeerApp::onStartStopRecording()
{
    if (mCameraHandle == CAMERA_HANDLE_INVALID) return;

    if (record)
    {
        record = false;

        onStopDecoder();

        ffe_context->stop();

        recordButton->setText("Record");
        stopCameraButton->setEnabled(true);

        return;
    }

    client_ready = false;

    fprintf(stderr, "Connecting...\n");
    if (!client->start())
    {
        fprintf(stderr, "Cannot connect to server\n");
        return;
    }

    if (!start_encoder(CODEC_ID)) return;

//    if (!start_decoder(CODEC_ID))
//    {
//        ffe_context->stop();
//        ffe_context->close();
//        return;
//    }

    decode = true;
    record = true;

    recordButton->setText("Stop");
    stopCameraButton->setEnabled(false);
}

bool CommandeerApp::start_decoder(CodecID codec_id)
{
    AVCodec *codec = avcodec_find_decoder(codec_id);

    if (!codec)
    {
        av_register_all();
        codec = avcodec_find_decoder(codec_id);

        if (!codec)
        {
            fprintf(stderr, "could not find codec\n");
            return false;
        }
    }

    AVCodecContext *codec_context = avcodec_alloc_context3(codec);
    codec_context->pix_fmt = PIX_FMT_YUV420P;
    codec_context->width = VIDEO_WIDTH;
    codec_context->height = VIDEO_HEIGHT;
    codec_context->thread_count = 2;

    if (codec->capabilities & CODEC_CAP_TRUNCATED)
    {
        // we do not send complete frames
        codec_context->flags |= CODEC_FLAG_TRUNCATED;
    }

    ffd_context->reset();
    ffd_context->set_close_callback(ffd_context_close, this);
    ffd_context->set_read_callback(ffd_read_callback, this);
    ffd_context->set_frame_callback(ffd_frame_callback, this);
    ffd_context->codec_context = codec_context;

    if (avcodec_open2(codec_context, codec, 0) < 0)
    {
        av_free(codec_context);
        fprintf(stderr, "could not open codec context\n");
        return false;
    }

    screen_window_t window;
    ffd_context->create_view(Application::instance()->mainWindow()->groupId(), "playbackWindow", &window);

    int window_size[] = { 768, 1280 };
    screen_set_window_property_iv(window, SCREEN_PROPERTY_SIZE, window_size);

    int z = -2;
    screen_set_window_property_iv(window, SCREEN_PROPERTY_ZORDER, &z);

    if (ffd_context->start() != FFDEC_OK)
    {
        fprintf(stderr, "could not start ffdec\n");
        ffd_context->close();
        return false;
    }

    qDebug() << "started ffdec_context";

    return true;
}

bool CommandeerApp::start_encoder(CodecID codec_id)
{
    AVCodec *codec = avcodec_find_encoder(codec_id);

    if (!codec)
    {
        av_register_all();
        codec = avcodec_find_encoder(codec_id);

        if (!codec)
        {
            fprintf(stderr, "could not find codec\n");
            return false;
        }
    }

    AVCodecContext *codec_context = avcodec_alloc_context3(codec);
    codec_context->pix_fmt = PIX_FMT_YUV420P;
    codec_context->width = VIDEO_WIDTH;
    codec_context->height = VIDEO_HEIGHT;
    codec_context->bit_rate = 400000;
    codec_context->time_base.num = 1;
    codec_context->time_base.den = 30;
    codec_context->ticks_per_frame = 2;
    codec_context->gop_size = 15;
    codec_context->colorspace = AVCOL_SPC_SMPTE170M;
    codec_context->thread_count = 2;

    ffe_context->reset();
    ffe_context->set_close_callback(ffe_context_close, this);
    ffe_context->set_write_callback(ffe_write_callback, this);
    ffe_context->set_frame_callback(ffe_frame_callback, this);
    ffe_context->codec_context = codec_context;

    frame_indices.clear();

    if (avcodec_open2(codec_context, codec, 0) < 0)
    {
        av_free(codec_context);
        fprintf(stderr, "could not open codec context\n");
        return false;
    }

    if (ffe_context->start() != FFENC_OK)
    {
        fprintf(stderr, "could not start ffenc\n");
        ffe_context->close();
        return false;
    }

    qDebug() << "started ffe_context";

    return true;
}

void vf_callback(camera_handle_t handle, camera_buffer_t* buf, void* arg)
{
    if (buf->frametype != CAMERA_FRAMETYPE_NV12) return;

    CommandeerApp* app = (CommandeerApp*) arg;

    long timestamp = hbc::current_time_millis();
    app->update_fps(app->fps_camera, timestamp);
    if (app->client_ready) app->print_fps(timestamp);

    app->ffe_context->add_frame(buf);
}

int ffd_read_callback(ffdec_context *ffd_context, uint8_t *buf, ssize_t size, void *arg)
{
    CommandeerApp* app = (CommandeerApp*) arg;

    if (!app->client_ready)
    {
        pthread_mutex_lock(&app->connected_mutex);
        if (!app->client_ready) pthread_cond_wait(&app->connected_cond, &app->connected_mutex);
        pthread_mutex_unlock(&app->connected_mutex);
    }

    cmdr::cmdr_client *cmdr_client = app->client;
    int r = ::read(cmdr_client->data_channel, buf, size * sizeof(uint8_t));
    app->bps_tracker_dec->update(r);
    return r;
}

bool ffe_frame_callback(ffenc_context *ffe_context, AVFrame *frame, int index, void *arg)
{
    CommandeerApp* app = (CommandeerApp*) arg;

    if (app->frame_indices.size() == 15) return false;

//    float frame_duration = 1000 / (float) app->frame_rate;
//
    long timestamp = hbc::current_time_millis();
//    static long last_timestamp = 0;
//    if (last_timestamp != 0 && timestamp - last_timestamp < frame_duration)
//    {
//        return false;
//    }
//
//    last_timestamp = timestamp;

    app->update_fps(app->fps_enc, timestamp);
    app->frame_indices.insert(std::pair<int, long>(index, timestamp));
    return true;
}

void frame_index_callback(int index, void* cookie)
{
    CommandeerApp* app = (CommandeerApp*) cookie;

    std::map<int, long>::iterator i = app->frame_indices.find(index);

    static int last = 0;
    if (index < last)
    {
        fprintf(stderr, "Frame index[%d] received out of order, last[%d]\n", index, last);
    }
    last = index;

    if (i == app->frame_indices.end())
    {
        fprintf(stderr, "Could not find timestamp for frame index[%d] received\nAvailable: \n", index);
        i = app->frame_indices.begin();
        for (; i != app->frame_indices.end(); i++)
        {
            if (i != app->frame_indices.begin()) fprintf(stderr, ", ");
            fprintf(stderr, "%d", i->first);
        }
        fprintf(stderr, "\n");
    }
    else app->frame_indices.erase(i);
}

void ffe_write_callback(ffenc_context *ffe_context, uint8_t *buf, ssize_t size, void *arg)
{
    CommandeerApp* app = (CommandeerApp*) arg;

    if (!app->client_ready)
    {
        pthread_mutex_lock(&app->connected_mutex);
        if (!app->client_ready) pthread_cond_wait(&app->connected_cond, &app->connected_mutex);
        pthread_mutex_unlock(&app->connected_mutex);
    }

    cmdr::cmdr_client *cmdr_client = app->client;

    int o = 0;
    do
    {
        int w = ::write(cmdr_client->data_channel, &buf[o], (size - o) * sizeof(uint8_t));
        if (w < 0) break;
        o += w;
    }
    while (o < size);

    app->bps_tracker_enc->update(o);
}

void ffd_frame_callback(ffdec_context *ffd_context, AVFrame *frame, int index, void *arg)
{
    CommandeerApp* app = (CommandeerApp*) arg;
    app->update_fps(app->fps_dec, hbc::current_time_millis());
}

void CommandeerApp::update_fps(std::deque<long> &fps, long timestamp)
{
    fps.push_back(timestamp);
    while (timestamp - fps.front() >= 1000)
    {
        fps.pop_front();
    }
}

void CommandeerApp::print_fps(long timestamp)
{
    static long last_timestamp = timestamp;
    if (timestamp - last_timestamp < 1000)
    {
        return;
    }

    last_timestamp = timestamp;

    long longest_wait = 0;

    if (frame_indices.size())
    {
        std::map<int, long>::iterator i;
        for (i = frame_indices.begin(); i != frame_indices.end(); i++)
        {
            if (!longest_wait) longest_wait = i->second;
            longest_wait = std::min(longest_wait, i->second);
        }

        longest_wait = timestamp - longest_wait;
    }

    double w;
    cmdr::bps_type w_typ = bps_tracker_enc->get_bps(&w);

    double r;
    cmdr::bps_type r_typ = bps_tracker_dec->get_bps(&r);

    fprintf(stderr, "fps_camera[%d], fps_dec[%d], fps_enc[%d], read[%.2f%s], write[%.2f%s], in_transit[%d], wait[%ldms]\n",
    fps_camera.size(), fps_dec.size(), fps_enc.size(),
    r_typ == cmdr::none ? 0 : r, cmdr::bps_type_str(r_typ),
    w_typ == cmdr::none ? 0 : w, cmdr::bps_type_str(w_typ),
    frame_indices.size(), longest_wait);
}

void ffe_context_close(ffenc_context *ffe_context, void *arg)
{
    qDebug() << "closing ffenc_context";

    ffe_context->close();

    CommandeerApp* app = (CommandeerApp*) arg;
    app->client_ready = false;
    app->record = false;

    cmdr::cmdr_client *cmdr_client = app->client;
    cmdr_client->disconnect();

    app->frame_indices.clear();
}

void ffd_context_close(ffdec_context *ffd_context, void *arg)
{
    qDebug() << "closing ffdec_context";

    ffd_context->close();

    CommandeerApp* app = (CommandeerApp*) arg;
    app->decode = false;
}
