#include "../include/Cell.h"

#include <iostream>
#include <sstream>

namespace algorithm {

Cell::Cell()
    : Cell(0,0) {
  // No op.
}

Cell::Cell(int x, int y)
  : x_(x),
    y_(y),
    visited_(false),
    mapped_(false),
    parent_(nullptr),
    has_left_(true),
    has_right_(true),
    has_top_(true),
    has_bottom_(true) {
  // No op.
}

void Cell::Configure(int x, int y) {
  x_ = x;
  y_ = y;
}

void Cell::Print() {
  std::stringstream ss;
  ss << "--------------------\n";
  ss << "Printing cell: ";
  ss << x_;
  ss << ",";
  ss << y_;
  ss << "\n";
  ss << "It is ";
  if (!visited_) {
    ss << "not ";
  }
  ss << "visited\n";
  ss << "There is ";
  if (!has_left_) {
    ss << "not ";
  }
  ss << "a wall to the left\n";
  ss << "There is ";
  if (!has_right_) {
    ss << "not ";
  }
  ss << "a wall to the right\n";
  ss << "There is ";
  if (!has_top_) {
    ss << "not ";
  }
  ss << "a wall to the top\n";
  ss << "There is ";
  if (!has_bottom_) {
    ss << "not ";
  }
  ss << "a wall to the bottom\n";
  std::cout << ss.str();
}

}
