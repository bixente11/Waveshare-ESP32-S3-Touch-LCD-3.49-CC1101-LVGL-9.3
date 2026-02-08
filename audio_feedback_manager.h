#pragma once

void audio_feedback_init();

void audio_feedback_play_startup();
void audio_feedback_play_shutdown();
void audio_feedback_play_detect();

bool audio_feedback_is_idle();
