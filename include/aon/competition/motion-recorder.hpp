#pragma once

#include <cstdio>
#include <stdint.h>
#include "pros/rtos.h"
#include "pros/motors.hpp"
#include "pros/misc.hpp"
#include "aon/constants.hpp"

// ============================================================================
//   Motion Recorder — graba tus movimientos manuales y los reproduce en auton
// ============================================================================
//
//  CÓMO USAR:
//
//  1. En opcontrol, agrega esto donde quieras activar la grabación:
//
//       if (mainController.get_digital_new_press(DIGITAL_X)) {
//           if (!rec::isRecording()) rec::start();
//           else                     rec::stop();
//       }
//       rec::captureFrame();  // <-- al final del loop de opcontrol (cada 20ms)
//
//  2. Maneja el robot como normalmente lo harías durante 15 segundos.
//
//  3. Presiona X de nuevo para parar. Se guarda en /usd/motion.csv
//
//  4. En tu rutina de autónomo llama:
//
//       rec::replay();
//
//  Eso es todo. No toca nada de lo que ya existe.
//
//  NOTA: No incluyas este archivo junto con noarmbot.hpp — ambos definen
//        el namespace rec y causarían conflicto.

#ifdef _PROS_LLEMU_H_
#  define REC_LCD(line, ...) pros::c::lcd_print(line, __VA_ARGS__)
#else
#  define REC_LCD(line, ...) (void(0))
#endif

namespace rec {

static const int   MAX_FRAMES = 750;
static const char* FILE_PATH  = "/usd/motion.csv";

struct Frame {
  uint16_t dt_ms;
  int16_t  leftRPM;
  int16_t  rightRPM;
};

static Frame    g_frames[MAX_FRAMES];
static int      g_count     = 0;
static bool     g_recording = false;
static uint32_t g_lastTime  = 0;

bool isRecording() { return g_recording; }

void start() {
  g_count     = 0;
  g_recording = true;
  g_lastTime  = pros::millis();
  REC_LCD(0, "REC grabando...");
  REC_LCD(1, "Presiona X para parar");
}

void captureFrame() {
  if (!g_recording || g_count >= MAX_FRAMES) return;

  uint32_t now = pros::millis();
  uint16_t dt  = (uint16_t)(now - g_lastTime);
  g_lastTime   = now;

#if USING_BIG_ROBOT
  pros::Motor lm(13);   // front-left
  pros::Motor rm(-1);   // front-right (reversed)
#else
  pros::Motor lm(11);
  pros::Motor rm(1);
#endif

  g_frames[g_count++] = {
    dt,
    (int16_t)lm.get_actual_velocity(),
    (int16_t)rm.get_actual_velocity()
  };

  if (g_count % 50 == 0) {
    REC_LCD(0, "REC %ds grabados", g_count / 50);
  }
}

void stop() {
  g_recording = false;
  REC_LCD(0, "Guardando...");

  if (!pros::usd::is_installed()) {
    REC_LCD(1, "ERROR: no hay SD card");
    return;
  }

  FILE* fp = fopen(FILE_PATH, "w");
  if (fp == nullptr) {
    REC_LCD(1, "ERROR: no se pudo abrir archivo");
    return;
  }

  fprintf(fp, "dt_ms,leftRPM,rightRPM\n");
  for (int i = 0; i < g_count; i++) {
    fprintf(fp, "%u,%d,%d\n",
            g_frames[i].dt_ms,
            (int)g_frames[i].leftRPM,
            (int)g_frames[i].rightRPM);
  }
  fclose(fp);

  REC_LCD(0, "Guardado: %d frames", g_count);
  REC_LCD(1, "Duracion: %.1fs", g_count * 0.02f);
  REC_LCD(2, "Archivo: /usd/motion.csv");
}

void replay() {
  if (!pros::usd::is_installed()) {
    REC_LCD(0, "ERROR replay: no SD card");
    return;
  }

  FILE* fp = fopen(FILE_PATH, "r");
  if (fp == nullptr) {
    REC_LCD(0, "ERROR replay: no hay grabacion");
    REC_LCD(1, "Corre opcontrol primero");
    return;
  }

  char line[64];
  fgets(line, 64, fp); // saltar cabecera

  int frameNum = 0;
  REC_LCD(0, "Reproduciendo...");

#if USING_BIG_ROBOT
  pros::Motor lm(13);
  pros::Motor rm(-1);
#else
  pros::Motor lm(11);
  pros::Motor rm(1);
#endif

  while (fgets(line, 64, fp)) {
    unsigned dt;
    int leftRPM, rightRPM;

    if (sscanf(line, "%u,%d,%d", &dt, &leftRPM, &rightRPM) == 3) {
      lm.move_velocity(leftRPM);
      rm.move_velocity(rightRPM);
      pros::delay(dt);
      frameNum++;

      if (frameNum % 50 == 0) {
        REC_LCD(1, "Frame %d / ~%d", frameNum, g_count);
      }
    }
  }

  fclose(fp);
  lm.move_velocity(0);
  rm.move_velocity(0);
  REC_LCD(0, "Replay completo");
}

} // namespace rec

#undef REC_LCD
