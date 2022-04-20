#pragma once

enum ZumoState : uint8_t {
  // Wall-E vil alltid være i én av disse tilstandene
  RESET = 0,
  CALIBRATE_LINESENSORS,
  WAIT_FOR_START_SIGNAL,
  MOVING,
  BRANCH_FOUND,
  MAP_BRANCHPOINT,
  RETURN_TO_STATION,
  BRAKING,
  STOPPED,
  REFUELING,
  SPIRALLING,
  PID_TUNE
};


String zumo_state_to_str(ZumoState state) {
  switch (state) {
    case ZumoState::RESET:
      return "RESET";
    case ZumoState::CALIBRATE_LINESENSORS:
      return "CALIBRATE_LINESENSORS";
    case ZumoState::WAIT_FOR_START_SIGNAL:
      return "WAIT_FOR_START_SIGNAL";
    case ZumoState::MOVING:
      return "MOVING";
    case ZumoState::BRANCH_FOUND:
      return "BRANCH_FOUND";
    case ZumoState::MAP_BRANCHPOINT:
      return "MAP_BRANCHPOINT";
    case ZumoState::RETURN_TO_STATION:
      return "RETURN_TO_STATION";
    case ZumoState::BRAKING:
      return "BRAKING";
    case ZumoState::STOPPED:
      return "STOPPED";
    case ZumoState::REFUELING:
      return "REFUELING";
    case ZumoState::SPIRALLING:
      return "SPIRALLING";
    case ZumoState::PID_TUNE:
      return "PID_TUNE";

    default:
      return "unknown state";
  }
}