#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define APP_FS_HZ           16000U

#define APP_FRAME_MS        1000U
#define APP_FRAME_LEN       16000U

#define APP_OVERLAP_LEN     8000U
#define APP_HOP_LEN         8000U

#define APP_CHIRP_MS        500U
#define APP_CHIRP_LEN       8000U

#define APP_CHIRP_F0_HZ     3000.0f
#define APP_CHIRP_F1_HZ     7000.0f

#define APP_CORR_LEN        (APP_FRAME_LEN - APP_CHIRP_LEN + 1U)

#define APP_DETECH_TH       0.60f

#define APP_WAV_FILENAME    "test_rl_1.wav"

#endif
