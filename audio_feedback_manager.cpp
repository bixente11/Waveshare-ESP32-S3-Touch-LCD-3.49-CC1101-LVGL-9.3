#include "audio_feedback_manager.h"

#include "codec_board.h"
#include "codec_init.h"
#include "esp_codec_dev.h"

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr const char *kBoardType = "S3_LCD_3_49";
constexpr int kSampleRate = 16000;
constexpr int kChannels = 2;
constexpr int kBitsPerSample = 16;
constexpr int kChunkFrames = 256;
constexpr float kPi = 3.14159265358979323846f;

enum class AudioEvent : uint8_t {
    Startup,
    Shutdown,
    Detect,
};

QueueHandle_t g_audio_queue = nullptr;
TaskHandle_t g_audio_task = nullptr;
volatile bool g_playing = false;

bool g_codec_ready = false;
esp_codec_dev_handle_t g_playback = nullptr;

bool codec_prepare() {
    if (g_codec_ready && g_playback) {
        return true;
    }

    set_codec_board_type(kBoardType);

    codec_init_cfg_t cfg{};
    cfg.in_mode = CODEC_I2S_MODE_NONE;
    cfg.out_mode = CODEC_I2S_MODE_STD;
    cfg.in_use_tdm = false;
    cfg.reuse_dev = true;

    if (init_codec(&cfg) != 0) {
        Serial.println("[AUDIO] init_codec failed");
        return false;
    }

    g_playback = get_playback_handle();
    if (!g_playback) {
        Serial.println("[AUDIO] playback handle unavailable");
        return false;
    }

    const int vol_ret = esp_codec_dev_set_out_vol(g_playback, 95);
    const int mute_ret = esp_codec_dev_set_out_mute(g_playback, false);
    const int close_cfg_ret = esp_codec_set_disable_when_closed(g_playback, false);
    Serial.printf("[AUDIO] codec ready vol=%d mute=%d close_cfg=%d\n", vol_ret, mute_ret, close_cfg_ret);
    g_codec_ready = true;
    return true;
}

void play_tone(float frequency_hz, int duration_ms, float gain) {
    if (!g_playback || frequency_hz <= 0.0f || duration_ms <= 0) {
        return;
    }

    const int total_frames = (kSampleRate * duration_ms) / 1000;
    const int fade_frames = (kSampleRate * 8) / 1000;

    int16_t pcm[kChunkFrames * kChannels];
    float phase = 0.0f;
    const float phase_step = 2.0f * kPi * frequency_hz / static_cast<float>(kSampleRate);

    int written = 0;
    while (written < total_frames) {
        const int chunk = (total_frames - written > kChunkFrames) ? kChunkFrames : (total_frames - written);

        for (int i = 0; i < chunk; ++i) {
            const int idx = written + i;
            float env = 1.0f;

            if (idx < fade_frames) {
                env = static_cast<float>(idx) / static_cast<float>(fade_frames);
            } else if (idx > total_frames - fade_frames) {
                env = static_cast<float>(total_frames - idx) / static_cast<float>(fade_frames);
            }

            const float sample_f = sinf(phase) * gain * env;
            const int16_t sample = static_cast<int16_t>(sample_f * 32767.0f);

            pcm[i * 2] = sample;
            pcm[i * 2 + 1] = sample;

            phase += phase_step;
            if (phase > 2.0f * kPi) {
                phase -= 2.0f * kPi;
            }
        }

        const int wr = esp_codec_dev_write(g_playback, pcm, chunk * kChannels * sizeof(int16_t));
        if (wr != 0) {
            Serial.printf("[AUDIO] write failed: %d\n", wr);
            return;
        }
        written += chunk;
    }
}

void play_startup_sound() {
    Serial.println("[AUDIO] startup tone1");
    play_tone(660.0f, 130, 0.42f);
    vTaskDelay(pdMS_TO_TICKS(20));
    Serial.println("[AUDIO] startup tone2");
    play_tone(990.0f, 170, 0.40f);
}

void play_shutdown_sound() {
    Serial.println("[AUDIO] shutdown tone1");
    play_tone(880.0f, 70, 0.38f);
    vTaskDelay(pdMS_TO_TICKS(10));
    Serial.println("[AUDIO] shutdown tone2");
    play_tone(520.0f, 110, 0.40f);
}

void play_detect_sound() {
    play_tone(1400.0f, 45, 0.32f);
    vTaskDelay(pdMS_TO_TICKS(8));
    play_tone(1700.0f, 45, 0.30f);
}

void audio_task(void *arg) {
    (void)arg;

    esp_codec_dev_sample_info_t fs{};
    fs.sample_rate = kSampleRate;
    fs.channel = kChannels;
    fs.bits_per_sample = kBitsPerSample;

    for (;;) {
        AudioEvent evt;
        if (xQueueReceive(g_audio_queue, &evt, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!codec_prepare()) {
            continue;
        }

        g_playing = true;
        const int open_ret = esp_codec_dev_open(g_playback, &fs);
        if (open_ret != 0) {
            Serial.printf("[AUDIO] esp_codec_dev_open failed: %d\n", open_ret);
            g_playing = false;
            continue;
        }
        const int mute_ret = esp_codec_dev_set_out_mute(g_playback, false);
        const int vol_ret = esp_codec_dev_set_out_vol(g_playback, 95);
        Serial.printf("[AUDIO] stream opened mute=%d vol=%d\n", mute_ret, vol_ret);

        if (evt == AudioEvent::Startup) {
            play_startup_sound();
            Serial.println("[AUDIO] startup sound done");
        } else if (evt == AudioEvent::Shutdown) {
            play_shutdown_sound();
            Serial.println("[AUDIO] shutdown sound done");
        } else {
            play_detect_sound();
        }

        int16_t silence[kChunkFrames * kChannels] = {0};
        esp_codec_dev_write(g_playback, silence, sizeof(silence));
        const int close_ret = esp_codec_dev_close(g_playback);
        Serial.printf("[AUDIO] stream closed ret=%d\n", close_ret);
        g_playing = false;
    }
}

void enqueue_event(AudioEvent evt) {
    if (!g_audio_queue) {
        return;
    }

    if (xQueueSend(g_audio_queue, &evt, 0) != pdTRUE) {
        AudioEvent dropped;
        xQueueReceive(g_audio_queue, &dropped, 0);
        xQueueSend(g_audio_queue, &evt, 0);
    }
}

}  // namespace

void audio_feedback_init() {
    if (g_audio_task) {
        return;
    }

    g_audio_queue = xQueueCreate(4, sizeof(AudioEvent));
    if (!g_audio_queue) {
        Serial.println("[AUDIO] queue creation failed");
        return;
    }

    xTaskCreatePinnedToCore(
        audio_task,
        "audio_feedback_task",
        12288,
        nullptr,
        2,
        &g_audio_task,
        0
    );
}

void audio_feedback_play_startup() {
    enqueue_event(AudioEvent::Startup);
}

void audio_feedback_play_shutdown() {
    enqueue_event(AudioEvent::Shutdown);
}

void audio_feedback_play_detect() {
    enqueue_event(AudioEvent::Detect);
}

bool audio_feedback_is_idle() {
    if (!g_audio_queue) {
        return true;
    }
    return (!g_playing) && (uxQueueMessagesWaiting(g_audio_queue) == 0);
}
