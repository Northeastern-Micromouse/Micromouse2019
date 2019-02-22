#include "../include/AlgoRobot.h"

#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>

namespace algorithm {

using algorithm::Direction;

Robot::Robot(bool enable_debugging, int maze_x, int maze_y, Direction orientation)
    : maze_(enable_debugging, maze_x, maze_y),
      orientation_(orientation),
      enable_debugging_(enable_debugging),
      curr_x_(0),
      curr_y_(0),
      driveSpeed_(0),
      turnSpeed_(0),
      top_left_goal_x_(2),
      top_left_goal_y_(2),
      bottom_right_goal_x_(2),
      bottom_right_goal_y_(2) {
  winslow_.init();
  winslow_.enableMotors();
  std::ifstream speedStream;
  speedStream.open("/home/debian/speed.txt");
  speedStream >> driveSpeed_;
  speedStream >> turnSpeed_;
  speedStream >> top_left_goal_x_;
  speedStream >> top_left_goal_y_;
  speedStream.close();
}

void Robot::Map() {
  Log("--------------- Start Mapping the Maze ---------------");
  assert(curr_x_ == 0 && curr_y_ == 0);

  maze_.Clear();

  VisitCurrentCell();

  while (!neighbors_.empty()) {
    CellPair cell = neighbors_.top();
    Log("Got the cell: " + std::to_string(cell.first_->x_) + "," + std::to_string(cell.first_->y_));
    Log("Added from cell: " + std::to_string(cell.second_.x_) + "," + std::to_string(cell.second_.y_));
    neighbors_.pop();
	


	
    // Move to where you were when you put the cell on the stack.
    std::vector<Direction> path = GetPath(&maze_.get(curr_x_, curr_y_), &cell.second_);
    PrintPath(path);
    for (auto direction : path) {
		std::cout << direction << std::endl;
      Move(direction);
    }

    // Move to the cell
	Log("Move to cell");
    Direction dir = GetDirection(cell.first_);
    Move(dir);
	Log("visit");
    if (!VisitCurrentCell()) {
		Log("Go Back");
      GoBack(dir);
    }
  }

  Log("Done mapping the maze. At location: " + std::to_string(curr_x_) + "," = std::to_string(curr_y_));
}

Direction Robot::GetDirection(Cell* start, Cell* end) {
  assert(start != nullptr);
  assert(end != nullptr);
  if (start->x_ == end->x_ && start->y_ == end->y_ + 1) {
    return Direction::SOUTH;
  } else if (start->x_ == end->x_ && start->y_ == end->y_ - 1) {
    return Direction::NORTH;
  } else if (start->x_ == end->x_ + 1 && start->y_ == end->y_) {
    return Direction::WEST;
  } else if (start->x_ == end->x_ - 1 && start->y_ == end->y_) {
    return Direction::EAST;
  } else 
	  if (start->x_ == end->x_ && start->y_ == end->y_) {
    return Direction::NONE;
  }
  std::cout << start->x_ << "," << start->y_ << "     " << end->x_ << "," << end->y_ << std::endl;
  assert(false);
}

Direction Robot::GetDirection(Cell* cell) {
  return GetDirection(&maze_.get(curr_x_, curr_y_), cell);
}

void Robot::ComputeShortestPath() {
  Log("Computing the shortest path");
  // TODO(matt): Fix this hack.
  path_ = GetPath(&maze_.get(0,0), &maze_.get(top_left_goal_x_, top_left_goal_y_));
}

void Robot::Run() {
  Log("Running the robot :)");
  while (!path_.empty()) {
    Move(path_.back());
    path_.pop_back();
  }
}

void Robot::Log(const std::string& log) {
  if (enable_debugging_) {
    std::cout << log << std::endl;
  }
}

bool Robot::VisitCurrentCell() {
  Cell& cell = maze_.get(curr_x_, curr_y_);
  bool should_move_forward = false;
  // Only visit a cell if it is not visited.
  if (!cell.mapped_) {
    Log("--------------- Visit Cell ---------------");
    Log("X: " + std::to_string(cell.x_));
    Log("Y: " + std::to_string(cell.y_));
    Log("--------------------------------------------");
    switch (orientation_) {
      case Direction::NORTH:
        winslow_.checkWallFront(&cell.has_top_);
        winslow_.checkWallLeft(&cell.has_left_);
        winslow_.checkWallRight(&cell.has_right_);
        cell.has_bottom_ = false;
        break;
      case Direction::SOUTH:
        winslow_.checkWallFront(&cell.has_bottom_);
        winslow_.checkWallLeft(&cell.has_right_);
        winslow_.checkWallRight(&cell.has_left_);
        cell.has_top_ = false;
        break;
      case Direction::EAST:
        winslow_.checkWallFront(&cell.has_right_);
        winslow_.checkWallLeft(&cell.has_top_);
        winslow_.checkWallRight(&cell.has_bottom_);
        cell.has_left_ = false;
        break;
      case Direction::WEST:
        winslow_.checkWallFront(&cell.has_left_);
        winslow_.checkWallLeft(&cell.has_bottom_);
        winslow_.checkWallRight(&cell.has_top_);
        cell.has_right_ = false;
        break;
      default:
        assert(false);
    }

    for (Cell* c : maze_.GetNeighbors(curr_x_, curr_y_)) {
      if (!c->mapped_) {
		  Log("Add " + std::to_string(c->x_) + "," + std::to_string(c->y_));
        neighbors_.push(CellPair(c, Cell(curr_x_, curr_y_)));
        should_move_forward = true;
      } else {
		  Log("Do not add " + std::to_string(c->x_) + "," + std::to_string(c->y_));
	  }
    }

    cell.mapped_ = true;
  } else {
    std::stringstream ss;
    ss << "Trying to visit cell: ";
    ss << curr_x_;
    ss << ",";
    ss << curr_y_;
    ss << " and it is already visited";
    Log(ss.str());
  }
  return should_move_forward;
}

void Robot::Move(Direction dir) {
  switch (dir) {
    case Direction::NORTH:
      Log("Move north");
      TurnNorth();
      curr_y_ += 1;
      break;
    case Direction::EAST:
      Log("Move east");
      TurnEast();
      curr_x_ += 1;
      break;
    case Direction::WEST:
      Log("Move west");
      TurnWest();
      curr_x_ -= 1;
      break;
    case Direction::SOUTH:
      Log("Move south");
      TurnSouth();
      curr_y_ -= 1;
      break;
    case Direction::NONE:
      Log("Do not need to move");
      return;
  }

  Log("Moved to cell: " + std::to_string(curr_x_) + "," + std::to_string(curr_y_));
  assert(curr_y_ >= 0 && curr_x_ >= 0 && curr_y_ < maze_.cols() && curr_x_ < maze_.rows());
  winslow_.pid_drive(180, driveSpeed_);
}

void Robot::TurnNorth(){
  switch (orientation_) {
    case Direction::NORTH:
      // No op.
      break;
    case Direction::EAST:
      Rotate(-90);
      break;
    case Direction::WEST:
      Rotate(90);
      break;
    case Direction::SOUTH:
      Rotate(180);
      break;
    case Direction::NONE:
      // No op.
      break;
  }
  orientation_ = Direction::NORTH;
}

void Robot::TurnEast() {
  switch (orientation_) {
    case Direction::NORTH:
      Rotate(90);
      break;
    case Direction::EAST:
      // No op.
      break;
    case Direction::WEST:
      Rotate(180);
      break;
    case Direction::SOUTH:
      Rotate(-90);
      break;
    case Direction::NONE:
      // No op.
      break;
  }
  orientation_ = Direction::EAST;
}

void Robot::TurnWest() {
  switch (orientation_) {
    case Direction::NORTH:
      Rotate(-90);
      break;
    case Direction::EAST:
      Rotate(180);
      break;
    case Direction::WEST:
      // No op.
      break;
    case Direction::SOUTH:
      Rotate(90);
      break;
    case Direction::NONE:
      // No op.
      break;
  }
  orientation_ = Direction::WEST;
}

void Robot::TurnSouth() {
  switch (orientation_) {
    case Direction::NORTH:
      Rotate(180);
      break;
    case Direction::EAST:
      Rotate(90);
      break;
    case Direction::WEST:
      Rotate(-90);
      break;
    case Direction::SOUTH:
      // No op.
      break;
    case Direction::NONE:
      // No op.
      break;
  }
  orientation_ = Direction::SOUTH;
}

void Robot::Rotate(int degrees) {
  winslow_.turn(degrees / 90, turnSpeed_);
}

void Robot::GoBack(Direction dir) {
  switch (dir) {
    case Direction::NORTH:
      Move(Direction::SOUTH);
      return;
    case Direction::SOUTH:
      Move(Direction::NORTH);
      return;
    case Direction::EAST:
      Move(Direction::WEST);
      return;
    case Direction::WEST:
      Move(Direction::EAST);
      return;
    case Direction::NONE:
      // No op.
      return;
  }
}

std::vector<Direction> Robot::GetPath(Cell* start, Cell* end) {
  Cell* curr = start;
  std::vector<Direction> path;
  path.push_back(Direction::NONE);
  std::stack<Cell*> cells_to_visit;

  while (curr->x_ != end->x_ || curr->y_ != end->y_) {
    // Get the reachable neighbors of this cell and add them to the stack.
    std::vector<Cell*> neighbors = maze_.GetNeighbors(curr->x_, curr->y_);
    if (!curr->visited_) {
      for (Cell* neighbor : neighbors) {
        if (!neighbor->visited_) {
          neighbor->parent_ = curr;
          cells_to_visit.push(neighbor);
        }
      }
    }
    curr->visited_ = true;
    // Set the parent and advance curr
    curr = cells_to_visit.top();
    cells_to_visit.pop();
    assert(curr != nullptr && end != nullptr);
  }

  end->parent_ = curr;

  // Build the path
  curr = end;
  while (curr->x_ != start->x_ || curr->y_ != start->y_) {
    path.insert(path.begin(), GetDirection(curr->parent_, curr));
    curr = curr->parent_;
  }

  // Reset shit.
  maze_.ClearVisitedAndParent();
  return path;
}

bool Robot::IsInsideGoal(int x, int y) {
  return (x >= top_left_goal_x_ && x <= bottom_right_goal_x_)
         && (y <= top_left_goal_y_ && y >= bottom_right_goal_y_);
}

bool Robot::IsInsideGoal() {
  return IsInsideGoal(curr_x_, curr_y_);
}

void Robot::PrintPath(const std::vector<Direction>& path) {
  for (auto dir : path) {
    std::cout << dir << std::endl;
  }
}

}
