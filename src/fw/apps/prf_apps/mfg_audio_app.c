/*
 * Copyright 2025 Core Devices LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "applib/app.h"
#include "applib/ui/app_window_stack.h"
#include "applib/ui/text_layer.h"
#include "applib/ui/window.h"
#include "kernel/pbl_malloc.h"
#include "board/board.h"
#include "process_management/pebble_process_md.h"
#include "process_state/app_state/app_state.h"
#include "drivers/audio.h"

typedef struct {
  Window window;

  TextLayer title;
  TextLayer status;
} AppData;

static const int16_t sine_wave_4k[] = {
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
  0, 32767, 0, -32768, 0, 32767, 0, -32768,
};

static void prv_audio_trans_handler(uint32_t *free_size) {
    uint32_t available_size = *free_size;
    while (available_size > sizeof(sine_wave_4k)) {
      available_size = audio_write(AUDIO, (void*)&sine_wave_4k[0], sizeof(sine_wave_4k));
    }
}

static void prv_play_audio(void) {
  audio_init(AUDIO);
  audio_start(AUDIO, prv_audio_trans_handler);
  audio_set_volume(AUDIO, 100);
}

static void prv_handle_init(void) {
  AppData *data = app_malloc_check(sizeof(AppData));

  app_state_set_user_data(data);

  Window *window = &data->window;
  window_init(window, "");
  window_set_fullscreen(window, true);

  TextLayer *title = &data->title;
  text_layer_init(title, &window->layer.bounds);
  text_layer_set_font(title, fonts_get_system_font(FONT_KEY_GOTHIC_24_BOLD));
  text_layer_set_text_alignment(title, GTextAlignmentCenter);
  text_layer_set_text(title, "AUDIO TEST");
  layer_add_child(&window->layer, &title->layer);

  app_window_stack_push(window, true /* Animated */);
}

static void s_main(void) {
  prv_handle_init();
  prv_play_audio();
  app_event_loop();
  audio_stop(AUDIO);
}

const PebbleProcessMd *mfg_audio_app_get_info(void) {
  static const PebbleProcessMdSystem s_app_info = {
      .common.main_func = &s_main,
      // UUID: c1479d03-5550-4444-b1e7-e2cbad0e5678
      .common.uuid = {0xc1, 0x47, 0x9d, 0x03, 0x55, 0x50, 0x44, 0x44, 0xb1, 0xe7, 0xe2, 0xcb, 0xad,
                      0x0e, 0x56, 0x78},
      .name = "MfgAudio",
  };
  return (const PebbleProcessMd *)&s_app_info;
}
