#include "Smoco.h"
#include <chrono>
#include <iostream>

static std::chrono::system_clock::time_point startTime;

uint32_t millis() {
    static bool firstTime = true;
    auto now = std::chrono::system_clock::now();
    if (firstTime) {
        startTime = now;
        firstTime = false;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(startTime - now).count();
}

// Smoco::Smoco(ACAN_T4 *canBus, uint8_t canID) : canBus(canBus), canID(canID) {}
Smoco::Smoco(int32_t initialPosition, float maxSpeed) : m_position(initialPosition), m_mockPosition(initialPosition), m_maxSpeed(maxSpeed) {}

// void Smoco::sync(CANMessage message) {
//     if (message.id >> 4 == 0x7F) {
//         // Update debug telemetry.
//         if ((message.id & 0xF) <= 8) ((uint64_t *)&m_debugTelemetry)[message.id & 0xF] = message.data64;
//         return;
//     }
//     if (message.id >> 4 != canID) return; // Not this SMoCo.

//     SmocoCANMessage *smocoCANMessage = (SmocoCANMessage *)message.data;
//     if (message.rtr) {
//         // Send missing parameter.
//         switch (message.id & 0xF) {
//         case SMOCO_MESSAGE_ID_SMOOTHING:
//             sendRampRate();
//             break;
//         case SMOCO_MESSAGE_ID_PID:
//             sendPID();
//             break;
//         case SMOCO_MESSAGE_ID_SOFT_LIMIT:
//             sendSoftLimitPosition();
//             break;
//         }
//     } else {
//         switch (message.id & 0xF) {
//         case SMOCO_MESSAGE_ID_POSITION:
//             m_position = smocoCANMessage->position.position;
//             m_velocity = smocoCANMessage->position.velocity;
//             m_current = smocoCANMessage->position.current;
//             // Get the bit at each position and convert it to a bool.
//             m_limitSwitchA = !!(smocoCANMessage->position.flags & (1 << 7));
//             m_limitSwitchB = !!(smocoCANMessage->position.flags & (1 << 6));
//             m_softLimitA = !!(smocoCANMessage->position.flags & (1 << 5));
//             m_softLimitB = !!(smocoCANMessage->position.flags & (1 << 4));
//             break;
//         case SMOCO_MESSAGE_ID_POSITION_CALIBRATED:
//             m_calibrated = true;
//             break;
//         case SMOCO_MESSAGE_ID_ERROR:
//             m_commandErrorID = smocoCANMessage->commandError.commandID;
//             break;
//         case SMOCO_MESSAGE_ID_ECHO_REPLY:
//             m_echoResponse = smocoCANMessage->echoReply.payload;
//             m_pingTime = millis() - m_echoResponse;
//             m_lastPingReply = millis();
//             break;
//         }
//     }
// }

bool Smoco::driveOpenLoop(int16_t dutyCycle) {
    bool ignoreLimit = false;
    if (dutyCycle > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (dutyCycle < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_dutyCycle = dutyCycle;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
    //                .len = 3,
    //                .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_OPEN_LOOP | ignoreLimit),
    //                                                     .openLoop = {.dutyCycle = m_dutyCycle}}}
    //                              .data64});

    // m_mockVelocity = (float)dutyCycle / INT16_MAX * m_maxSpeed;
    m_mode = CONTROL_MODE_OPEN_LOOP;

    return true;
}

bool Smoco::driveTargetPosition(int32_t targetPosition, float errorGain) {
    bool ignoreLimit = false;
    if (targetPosition > m_targetPosition) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetPosition < m_targetPosition) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetPosition = targetPosition;
    m_errorGain = errorGain;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
    //                .len = 7,
    //                .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_POSITION | ignoreLimit),
    //                                                     .targetPosition = {.errorGain = (uint16_t)(m_errorGain * 1024),
    //                                                                        .position = m_targetPosition}}}
    //                              .data64});

    // m_mockVelocity = 0;
    // m_mockPosition = targetPosition; // snap there for simulation purposes
    m_mode = CONTROL_MODE_POSITION;

    return true;
}

bool Smoco::driveTargetVelocity(int32_t targetVelocity, float errorGain) {
    bool ignoreLimit = false;
    if (targetVelocity > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetVelocity < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetVelocity = targetVelocity;
    m_errorGain = errorGain;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
    //                .len = 7,
    //                .data64 = SmocoCANMessage{.target = {.sid = (uint8_t)(SMOCO_MESSAGE_SID_VELOCITY | ignoreLimit),
    //                                                     .targetVelocity = {.errorGain = (uint16_t)(m_errorGain * 1024),
    //                                                                        .velocity = m_targetVelocity}}}
    //                              .data64});
    m_mode = CONTROL_MODE_VELOCITY;
    return true;
}

bool Smoco::driveTargetCurrent(float targetCurrent, float errorGain) {
    bool ignoreLimit = false;
    if (targetCurrent > 0) {
        ignoreLimit = m_ignoreForwardLimit;
    } else if (targetCurrent < 0) {
        ignoreLimit = m_ignoreReverseLimit;
    }

    m_targetCurrent = targetCurrent;
    m_errorGain = errorGain;
    // return canBus->tryToSend(CANMessage{
    //     .id = (canID << 4) | SMOCO_MESSAGE_ID_TARGET,
    //     .len = 5,
    //     .data64 = SmocoCANMessage{
    //         .target = {
    //             .sid = (uint8_t)(SMOCO_MESSAGE_SID_CURRENT | ignoreLimit),
    //             .targetCurrent = {
    //                 .errorGain = (uint16_t)(m_targetCurrent * 1024),
    //                 .current = (int16_t)(targetCurrent * 8) // TODO: Update scaling factor when current is supported
    //             }}}.data64});
    m_mode = CONTROL_MODE_CURRENT;
    return true;
}

void Smoco::configIgnoreLimits(bool forward, bool reverse) {
    m_ignoreForwardLimit = forward;
    m_ignoreReverseLimit = reverse;
}

bool Smoco::setRampRate(double rampRate) {
    m_rampRate = rampRate;
    return sendRampRate();
    return true;
}

bool Smoco::sendRampRate() {
    // return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_SMOOTHING,
    //                                     .len = 8,
    //                                     .data64 = SmocoCANMessage{.setRampRate = {.rampRate = m_rampRate}}.data64});
    return true;
}

bool Smoco::setPID(float P, float I, float D) {
    m_PID.P = P;
    m_PID.I = I;
    m_PID.D = D;
    return sendPID();
}

bool Smoco::sendPID() {
    // return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_PID,
    //                                     .len = 6,
    //                                     .data64 = SmocoCANMessage{.setPID = {.p = (uint16_t)(m_PID.P * 256),
    //                                                                          .i = (uint16_t)(m_PID.I * 256),
    //                                                                          .d = (uint16_t)(m_PID.D * 256)}}
    //                                                   .data64});
    return true;
}

bool Smoco::setSoftLimitPosition(int32_t positionA, int32_t positionB) {
    m_softLimitAPosition = positionA;
    m_softLimitBPosition = positionB;
    return sendSoftLimitPosition();
}

bool Smoco::sendSoftLimitPosition() {
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_SOFT_LIMIT,
    //                .len = 8,
    //                .data64 = SmocoCANMessage{.setSoftLimitPosition = {.aPosition = m_softLimitAPosition,
    //                                                                   .bPosition = m_softLimitBPosition}}
    //                              .data64});
    return true;
}

bool Smoco::calibratePosition(int16_t dutyCycle, int32_t limitSwitchPosition) {
    m_calibrationDutyCycle = dutyCycle;
    m_calibrationPosition = limitSwitchPosition;
    m_calibrated = false;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_CALIBRATE,
    //                .len = 6,
    //                .data64 = SmocoCANMessage{.startPositionCalibration = {.dutyCycle = m_calibrationDutyCycle,
    //                                                                       .limitSwitchPosition = m_calibrationPosition}}
    //                              .data64});
    return true;
}

bool Smoco::debugTelemetry(bool enable) {
    m_debugTelemetryEnabled = enable;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_DEBUG,
    //                .len = 1,
    //                .data64 = SmocoCANMessage{.debugTelemetry = {.enable = m_debugTelemetryEnabled}}.data64});
    return true;
}

bool Smoco::stopAndReset() {
    // return canBus->tryToSend(CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_STOP, .len = 0});
    return true;
}

bool Smoco::echoRequest(uint64_t payload) {
    m_echoRequestPayload = payload;
    // return canBus->tryToSend(
    //     CANMessage{.id = (canID << 4) | SMOCO_MESSAGE_ID_ECHO_REQUEST,
    //                .len = 8,
    //                .data64 = SmocoCANMessage{.echoRequestPayload = {.payload = m_echoRequestPayload}}.data64});
    return true;
}

bool Smoco::ping() {
    if (millis() > m_lastPingReply + pingTimeout) {
        m_pingTime = pingTimeout;
    };
    return echoRequest(millis());
}

static double pide(double error, double P, double I, double D, double errorGain,
                   double *lastError, double *integralError, double deltaT) {
  error *= errorGain;
  *integralError += error * deltaT;
  double out =
      P * error + I * *integralError + D * (error - *lastError) * deltaT;
  *lastError = error;
  return out;
}

static void ramp(double in, double *out, double rampRate, double deltaT) {
  if (in > *out + rampRate * deltaT) {
    *out += rampRate * deltaT;
  } else if (in < *out - rampRate * deltaT) {
    *out -= rampRate * deltaT;
  } else {
    *out = in;
  }
}

void Smoco::update(float dt) {

    switch (m_mode) {
        case CONTROL_MODE_OPEN_LOOP:
            m_mockVelocity = (float)m_dutyCycle * INT16_MAX / m_maxSpeed;
            break;
        case CONTROL_MODE_POSITION:
            m_mockVelocity = (float)(m_targetPosition - m_mockPosition) / m_maxSpeed;
            break;
    }

    double error;
    switch (m_mode) {
    case CONTROL_MODE_OPEN_LOOP:
      m_pwm = m_pwm < -1 ? -1 : m_pwm > 1 ? 1 : m_pwm;
      ramp((double)m_dutyCycle / 32768, &m_pwm, m_rampRate, dt);
      break;
    case CONTROL_MODE_POSITION:
      error = (float)m_targetPosition - m_mockPosition;
      m_pwm = pide(error, m_PID.P, m_PID.I, m_PID.D, m_errorGain, &m_lastError, &m_integralError, dt);
      break;
    case CONTROL_MODE_VELOCITY:
      error = (float)m_targetVelocity - m_mockVelocity;
      m_pwm = pide(error, m_PID.P, m_PID.I, m_PID.D, m_errorGain, &m_lastError, &m_integralError, dt);
      break;
    case CONTROL_MODE_CURRENT:
      error = (float)m_targetCurrent - m_current;
      m_pwm = pide(error, m_PID.P, m_PID.I, m_PID.D, m_errorGain, &m_lastError, &m_integralError, dt);
      break;
    default:
      break;
    }

    // clamp to -1..1
    m_pwm = m_pwm < -1 ? -1 : m_pwm > 1 ? 1 : m_pwm;

    m_softLimitA = m_mockPosition <= m_softLimitAPosition;
    m_softLimitB = m_mockPosition >= m_softLimitBPosition;
    if (m_pwm < 0 && m_softLimitA || m_pwm > 0 && m_softLimitB) {
        m_pwm = 0;
    }

    m_mockVelocity = m_pwm * m_maxSpeed;
    m_mockPosition += m_mockVelocity * dt;

    m_velocity = m_mockVelocity;
    m_position = m_mockPosition;
}
