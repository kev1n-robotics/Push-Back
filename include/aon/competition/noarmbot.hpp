#pragma once

#include <cstdio>
#include <vector>
#include <algorithm>
#include "aon/constants.hpp"

// Este archivo contiene toda la lógica de la rutina noarmBot:
//   - safeTurnToHeading  (giro con fallback a encoders)
//   - driveStraight      (avance con corrección de heading)
//   - turnTo2Stage       (giro en 2 etapas para mayor precisión)
//   - Run Logger         (registro de error real vs esperado en /usd/auton_log.txt)
//   - noarmBot           (la rutina en sí)
//
// Se incluye desde autonomous-routines.hpp dentro de namespace aon.
// Todos los globals (drivetrain, odometry, intake, mainController, etc.)
// están disponibles porque autonomous-routines.hpp ya los incluye.
//
// NOTA: Este archivo incluye su propia copia del motion recorder (namespace rec).
//       No incluyas motion-recorder.hpp en la misma unidad de compilación.

// ============================================================================
//   safeTurnToHeading
// ============================================================================

/// @brief Gira al heading absoluto usando el giroscopio.
///        Si el giroscopio deja de responder, usa encoders de los motores.
/// @param targetHeading Heading en grados al que queremos llegar
void safeTurnToHeading(double targetHeading) {
  if (targetHeading != targetHeading) return;
  const double TOLERANCE_DEG = 3.0;
  const double FALLBACK_RPM  = 200;
  const double TIMEOUT_MS    = 3000;
  const double STUCK_MS      = 400;

  double lastGyroVal      = drivetrain.getTheta();
  double lastGyroChangeMs = pros::millis();
  bool   gyroFailed       = false;

#if USING_BIG_ROBOT
  pros::Motor leftMotor(13);
  pros::Motor rightMotor(-1);
#else
  pros::Motor leftMotor(11);
  pros::Motor rightMotor(1);
#endif
  double motorLeftStart  = leftMotor.get_position();
  double motorRightStart = rightMotor.get_position();

  double delta = targetHeading - lastGyroVal;
  if (delta >  180) delta -= 360;
  if (delta < -180) delta += 360;
  const int    dir        = (delta >= 0) ? 1 : -1;
  const double totalAngle = std::abs(delta);

  const double startMs = pros::millis();

  while (pros::millis() - startMs < TIMEOUT_MS) {
    double currentHeading = drivetrain.getTheta();

    if (std::abs(currentHeading - lastGyroVal) > 0.5) {
      lastGyroVal      = currentHeading;
      lastGyroChangeMs = pros::millis();
    }
    if (pros::millis() - lastGyroChangeMs > STUCK_MS) {
      gyroFailed = true;
    }

    if (!gyroFailed) {
      double error = targetHeading - currentHeading;
      if (error >  180) error -= 360;
      if (error < -180) error += 360;
      if (std::abs(error) < TOLERANCE_DEG) break;

      double speed = std::clamp(std::abs(error) * 3.0, 50.0, 300.0);
      drivetrain.rotate(dir * speed);
    } else {
      double leftDist  = ((leftMotor.get_position()  - motorLeftStart)  / 360.0) * M_PI * DRIVE_WHEEL_DIAMETER * MOTOR_TO_DRIVE_RATIO;
      double rightDist = ((rightMotor.get_position() - motorRightStart) / 360.0) * M_PI * DRIVE_WHEEL_DIAMETER * MOTOR_TO_DRIVE_RATIO;
      double estimatedAngle = std::abs((leftDist - rightDist) / DRIVE_WIDTH * (180.0 / M_PI));
      if (estimatedAngle >= totalAngle) break;
      drivetrain.rotate(dir * FALLBACK_RPM);
    }
    pros::delay(20);
  }
  drivetrain.stop();
}

// ============================================================================
//   driveStraight
// ============================================================================

/// @param dist_in   Pulgadas (positivo = adelante, negativo = reversa)
/// @param base_pct  Velocidad base 0-100
/// @param kP        Corrección de heading
/// @param slow_in   Pulgadas antes del objetivo donde empieza a frenar
void driveStraight(double dist_in, double base_pct = 50, double kP = 0.5, double slow_in = 5.0) {
  if (dist_in == 0) return;

  const int    sign      = dist_in > 0 ? 1 : -1;
  const double target_in = std::abs(dist_in);

  double startHeading = drivetrain.getTheta();
  double encStart     = odometry.encoderLeft.get_position();

#if USING_BIG_ROBOT
  pros::Motor leftMotor(13);
#else
  pros::Motor leftMotor(11);
#endif
  double motorStart = leftMotor.get_position();

  const double timeoutMs = (target_in / 3.0) * 1000.0 + 1000.0;
  const double startMs   = pros::millis();

  while (pros::millis() - startMs < timeoutMs) {
    double encDist   = std::abs(((odometry.encoderLeft.get_position() - encStart) / 36000.0) * M_PI * TRACKING_WHEEL_DIAMETER);
    double motorDist = (std::abs(leftMotor.get_position() - motorStart) / 360.0) * M_PI * DRIVE_WHEEL_DIAMETER * MOTOR_TO_DRIVE_RATIO;

    double traveled  = (encDist < 0.5 && motorDist > 1.0) ? motorDist : encDist;
    double remaining = target_in - traveled;
    if (remaining <= 0.1) break;

    double headingDelta = drivetrain.getTheta() - startHeading;
    if (headingDelta >  180) headingDelta -= 360;
    if (headingDelta < -180) headingDelta += 360;

    double trim     = std::clamp(kP * (-headingDelta), -15.0, 15.0);
    double pct      = (remaining < slow_in) ? std::max(25.0, base_pct * (remaining / slow_in)) : base_pct;
    double speedRPM = sign * (pct / 100.0) * MAX_RPM;
    double trimRPM  = (trim / 100.0) * MAX_RPM;

    drivetrain.arcade(speedRPM, trimRPM);
    pros::delay(20);
  }
  drivetrain.stop();
}

// ============================================================================
//   turnTo2Stage
// ============================================================================

/// @brief Gira al heading en dos etapas: rápido para la mayor parte,
///        lento para los últimos grados, para mayor precisión.
/// @param targetHeading Heading absoluto en grados
void turnTo2Stage(double targetHeading) {
  const double SPLIT = 0.92;

  double delta = targetHeading - drivetrain.getTheta();
  if (delta >  180) delta -= 360;
  if (delta < -180) delta += 360;
  drivetrain.turn(delta * SPLIT);

  pros::delay(120);

  double remaining = targetHeading - drivetrain.getTheta();
  if (remaining >  180) remaining -= 360;
  if (remaining < -180) remaining += 360;

  drivetrain.turnPID(turnPID, remaining, 25.0);

  pros::delay(120);
}

// ============================================================================
//   Run Logger
// ============================================================================

#ifdef _PROS_LLEMU_H_
#  define NOARM_LCD(line, ...) pros::c::lcd_print(line, __VA_ARGS__)
#else
#  define NOARM_LCD(line, ...) (void(0))
#endif

struct StepLog {
  char   label[40];
  double expected;
  double actual;
  char   unit[5];
};

std::vector<StepLog> g_runLog;
double               g_runBattery = 0;

void logClear() {
  g_runLog.clear();
  g_runBattery = pros::battery::get_voltage() / 1000.0;
}

void logStep(const char* label, double expected, double actual, const char* unit) {
  StepLog e;
  std::snprintf(e.label, sizeof(e.label), "%s", label);
  e.expected = expected;
  e.actual   = actual;
  std::snprintf(e.unit, sizeof(e.unit), "%s", unit);
  g_runLog.push_back(e);
}

void logSave() {
  NOARM_LCD(0, "=== AUTON LOG ===");
  for (int i = 0; i < (int)g_runLog.size() && i < 7; i++) {
    double err = g_runLog[i].actual - g_runLog[i].expected;
    NOARM_LCD(i + 1, "%s err:%+.2f%s", g_runLog[i].label, err, g_runLog[i].unit);
  }

  double worstErr = 0; int worstIdx = 0;
  for (int i = 0; i < (int)g_runLog.size(); i++) {
    double e = std::abs(g_runLog[i].actual - g_runLog[i].expected);
    if (e > worstErr) { worstErr = e; worstIdx = i; }
  }
  if (!g_runLog.empty()) {
    mainController.print(0, 0, "WORST:%+.1f%s",
      g_runLog[worstIdx].actual - g_runLog[worstIdx].expected,
      g_runLog[worstIdx].unit);
  }

  if (!pros::usd::is_installed()) { return; }
  FILE* fp = fopen("/usd/auton_log.txt", "a");
  if (fp == nullptr) { return; }
  fprintf(fp, "\n========== RUN (bat:%.2fV) ==========\n", g_runBattery);
  fprintf(fp, "  %-22s  %8s  %8s  %8s\n", "paso", "esperado", "real", "error");
  fprintf(fp, "  %-22s  %8s  %8s  %8s\n", "----------------------", "--------", "--------", "--------");
  for (const auto& e : g_runLog) {
    double err = e.actual - e.expected;
    fprintf(fp, "  %-22s  %7.2f%s  %7.2f%s  %+7.2f%s\n",
            e.label, e.expected, e.unit, e.actual, e.unit, err, e.unit);
  }
  fprintf(fp, "=====================================\n");
  fclose(fp);
}

void loggedDriveStraight(double dist_in, double base_pct = 50, double kP = 0.5, double slow_in = 5.0) {
  double encBefore  = odometry.encoderLeft.get_position();
  driveStraight(dist_in, base_pct, kP, slow_in);
  double actualDist = ((odometry.encoderLeft.get_position() - encBefore) / 36000.0) * M_PI * TRACKING_WHEEL_DIAMETER;
  if (dist_in < 0) actualDist = -std::abs(actualDist);
  char label[40];
  std::snprintf(label, 40, "move(%.0f\")", dist_in);
  logStep(label, dist_in, actualDist, "in");
  NOARM_LCD(7, "move err:%+.2fin", actualDist - dist_in);
}

void loggedTurnTo2Stage(double targetHeading) {
  turnTo2Stage(targetHeading);
  double actualHeading = drivetrain.getTheta();
  char label[40];
  std::snprintf(label, 40, "turn(%.0fdeg)", targetHeading);
  logStep(label, targetHeading, actualHeading, "deg");
  NOARM_LCD(7, "turn err:%+.2fdeg", actualHeading - targetHeading);
}

// ============================================================================
//   Motion Recorder (copia interna — no incluyas motion-recorder.hpp junto a este)
// ============================================================================

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
  NOARM_LCD(0, "REC grabando...");
  NOARM_LCD(1, "Presiona X para parar");
}

void captureFrame() {
  if (!g_recording || g_count >= MAX_FRAMES) return;

  uint32_t now = pros::millis();
  uint16_t dt  = (uint16_t)(now - g_lastTime);
  g_lastTime   = now;

#if USING_BIG_ROBOT
  pros::Motor lm(13);
  pros::Motor rm(-1);
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
    NOARM_LCD(0, "REC %ds grabados", g_count / 50);
  }
}

void stop() {
  g_recording = false;
  NOARM_LCD(0, "Guardando...");

  if (!pros::usd::is_installed()) {
    NOARM_LCD(1, "ERROR: no hay SD card");
    return;
  }

  FILE* fp = fopen(FILE_PATH, "w");
  if (fp == nullptr) {
    NOARM_LCD(1, "ERROR: no se pudo abrir archivo");
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

  NOARM_LCD(0, "Guardado: %d frames", g_count);
  NOARM_LCD(1, "Duracion: %.1fs", g_count * 0.02f);
  NOARM_LCD(2, "Archivo: /usd/motion.csv");
}

void replay() {
  if (!pros::usd::is_installed()) {
    NOARM_LCD(0, "ERROR replay: no SD card");
    return;
  }

  FILE* fp = fopen(FILE_PATH, "r");
  if (fp == nullptr) {
    NOARM_LCD(0, "ERROR replay: no hay grabacion");
    NOARM_LCD(1, "Corre opcontrol primero");
    return;
  }

  char line[64];
  fgets(line, 64, fp);

#if USING_BIG_ROBOT
  pros::Motor lm(13);
  pros::Motor rm(-1);
#else
  pros::Motor lm(11);
  pros::Motor rm(1);
#endif

  int frameNum = 0;
  NOARM_LCD(0, "Reproduciendo...");

  while (fgets(line, 64, fp)) {
    unsigned dt;
    int leftRPM, rightRPM;

    if (sscanf(line, "%u,%d,%d", &dt, &leftRPM, &rightRPM) == 3) {
      lm.move_velocity(leftRPM);
      rm.move_velocity(rightRPM);
      pros::delay(dt);
      frameNum++;

      if (frameNum % 50 == 0) {
        NOARM_LCD(1, "Frame %d / ~%d", frameNum, g_count);
      }
    }
  }

  fclose(fp);
  lm.move_velocity(0);
  rm.move_velocity(0);
  NOARM_LCD(0, "Replay completo");
}

} // namespace rec

// ============================================================================
//   noarmBot — la rutina
// ============================================================================

void noarmBot() {
  logClear();

  loggedDriveStraight(24, 50, 0.5, 5.0);
  loggedTurnTo2Stage(90);
  loggedDriveStraight(24, 50, 0.5, 5.0);

  pros::delay(500);
  intake.dropCart();

  loggedDriveStraight(-24, 50, 0.5, 5.0);

  logSave();
}

#undef NOARM_LCD
