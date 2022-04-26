#pragma once

enum ZumoState : uint8_t {
  // Wall-E vil alltid være i én av disse tilstandene
  RESET = 0,
  CALIBRATE_LINESENSORS,
  WAIT_FOR_START_SIGNAL,
  SEARCHING_FOR_BOX,
  SCANNING_FOR_BOX,
  FOUND_BOX,
  TURNING_TO_BOX,
  LOST_TRACK_OF_BOX,
  MOVING_TO_BOX,
  GRABBING_BOX,
  MOVE_TO_BORDER,
  RETURN_TO_STATION,
  STOPPED,
  REFUELING,
  FOLLOW_LINE
};


String zumo_state_to_str(ZumoState state) {
  switch (state) {
    case ZumoState::RESET:
      return "RESET";
    case ZumoState::CALIBRATE_LINESENSORS:
      return "CALIBRATE_LINESENSORS";
    case ZumoState::WAIT_FOR_START_SIGNAL:
      return "WAIT_FOR_START_SIGNAL";
    case ZumoState::SEARCHING_FOR_BOX:
      return "SEARCHING_FOR_BOX";
    case ZumoState::SCANNING_FOR_BOX:
      return "SCANNING_FOR_BOX";
    case ZumoState::FOUND_BOX:
      return "FOUND_BOX";
    case ZumoState::TURNING_TO_BOX:
      return "TURNING_TO_BOX";
    case ZumoState::LOST_TRACK_OF_BOX:
      return "LOST_TRACK_OF_BOX";
    case ZumoState::MOVING_TO_BOX:
      return "MOVING_TO_BOX";
    case ZumoState::GRABBING_BOX:
      return "GRABBING_BOX";
    case ZumoState::MOVE_TO_BORDER:
      return "MOVE_TO_BORDER";
    case ZumoState::RETURN_TO_STATION:
      return "RETURN_TO_STATION";
    case ZumoState::STOPPED:
      return "STOPPED";
    case ZumoState::REFUELING:
      return "REFUELING";
    case ZumoState::FOLLOW_LINE:
      return "FOLLOW_LINE";

    default:
      return "unknown state";
  }
}
