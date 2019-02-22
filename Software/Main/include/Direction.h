#ifndef MICROMOUSE_ALGO_DIRECTION_H
#define MICROMOUSE_ALGO_DIRECTION_H

#include <iostream>

namespace algorithm {

enum class Direction {
  NORTH,
  SOUTH,
  EAST,
  WEST,
  NONE,
};

inline std::ostream& operator<<(std::ostream& os, const Direction& dir) {
  switch(dir) {
    case Direction::NONE:
      os << "NONE";
      break;
    case Direction::NORTH:
      os << "NORTH";
      break;
    case Direction::SOUTH:
      os << "SOUTH";
      break;
    case Direction::WEST:
      os << "WEST";
      break;
    case Direction::EAST:
      os << "EAST";
      break;
  }
  return os;
}

}  // namespace algorithm

#endif //MICROMOUSE_ALGO_DIRECTION_H
