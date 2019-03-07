#include "../include/AlgoRobot.h"

#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <climits>
#include <unistd.h>

namespace algorithm {

using algorithm::Direction;

Robot::Robot(micromouse::Robot* robot, bool enable_debugging, int maze_x, int maze_y, Direction orientation)
    : maze_(enable_debugging, maze_x, maze_y),
      orientation_(orientation),
      enable_debugging_(enable_debugging),
      curr_x_(0),
      curr_y_(0),
      top_left_goal_x_(2),
      top_left_goal_y_(3),
      bottom_right_goal_x_(3),
      bottom_right_goal_y_(2),
      robot_(robot) {

}

bool inPath(std::vector<Cell*> path, Cell* c) {
  //printf("cell: %d, %d\n",  c->x_, c->y_);
  for (int i = 0; i < path.size(); i++) {
    //printf("list elem: %d, %d\n",  path[i]->x_, path[i]->y_);
    if ((c->x_ == path[i]->x_) && (c->y_ == path[i]->y_)) {
      return true;
    }
  }
  //printf("added to list\n");
  return false;
}

bool Robot::Map() {
  printf("starting map\n");
  curr_x_ = 0;
  curr_y_ = 0;
  std::stack<Cell*> cells_to_visit;
  Cell* curr = &maze_.get(curr_x_, curr_y_);
  cells_to_visit.push(curr);
  std::vector<Cell*> visited = {};

  while(!cells_to_visit.empty()) {
	printf("reading buttons\n");
	if(robot_->readButton1()) {
	  printf("Read button 1\n");
      Reset(false);
	  return true;
	}
	if(robot_->readButton2()) {
	  printf("Read button 2\n");
	  Reset(true);
	  return false;
	}
	 printf("doing stack stuff\n");
    curr = cells_to_visit.top();
    cells_to_visit.pop();
    std::vector<Cell*> tmppath = {curr};
	  printf("move\n");
    moveToPath(tmppath);
    printf("x: %d y:% d\n", curr_x_, curr_y_);
    if (curr->x_ == top_left_goal_x_ && curr->y_ == top_left_goal_y_) {
      Cell* hi = &maze_.get(top_left_goal_x_,top_left_goal_y_);
      while (hi->x_ != 0 || hi->y_ != 0) {
        printf("col: %d row: %d ->", hi->x_, hi->y_);
        hi = hi->actualParent_;
      }
      printf("\n");
    }
    VisitCurrentCell();
    visited.push_back(curr);
    for (auto neighbor: maze_.GetNeighbors(curr->x_, curr->y_)) {
      if (!inPath(visited, neighbor)) {
		    printf("adding to stack: %d %d\n", neighbor->x_, neighbor->y_);
        neighbor->actualParent_ = curr;
        cells_to_visit.push(neighbor);
      }
    }
  }
  printf("exiting mapping\n");
  return true;
}

void Robot::Reset(bool wipeMap) {
  curr_x_ = 0;
  curr_y_ = 0;
  orientation_ = Direction::NORTH;

  usleep(2000000);
  while(!(robot_->readButton1()));
  usleep(500000);
  robot_->reset();

  if(wipeMap) {
    maze_ = Maze(enable_debugging_, maze_.cols(), maze_.rows());
  }
}

std::vector<Cell*> Robot::ComputeShortestPath() {
  printf("computing shortest path ;) \n\n");
  std::stack<std::vector<Cell*>> cells_to_visit;
  curr_x_ = 0;
  curr_y_ = 0;
  std::vector<Cell*> tmppath = {&maze_.get(curr_x_ , curr_y_)};
  Cell* curr;
  cells_to_visit.push(tmppath);
  std::vector<std::vector<Cell*>> possiblePaths;
  std::vector<Cell*> shortestPath;

  while(!cells_to_visit.empty()) {
  //for (int i = 0; i < 10; i++) {
    //maze_.print();
    std::vector<Cell*> tmppath = cells_to_visit.top();
    cells_to_visit.pop();
    curr = tmppath.back();
    //printf("x: %d y: %d\n", curr->x_, curr->y_);
    //TODO: actually move
    //TODO: orientation should be set by moving

    curr_x_ = curr->x_;
    curr_y_ = curr->y_;
    //robot_->setxy(curr_x_, curr_y_);
    //if (tmppath.size() > 1) {
    //  orientation_ = GetDirection(tmppath[tmppath.size() - 2], tmppath[tmppath.size() - 1]);
    //  robot_->setorientation(orientation_);
    //} else {
    //  printf("one elem path\n");
    //  orientation_ = Direction::SOUTH;
    //  robot_->setorientation(Direction::SOUTH);
    //}
    //TODO dont hardcode this
    //moveToPath(tmppath);
    if (curr->x_ == top_left_goal_x_ && curr->y_ == top_left_goal_y_) {
        possiblePaths.push_back(std::vector<Cell*>(tmppath));
    }
    //printf("visiting curr cell %d %d\n", curr->x_, curr->y_);
    //VisitCurrentCell();
	  //printf("neighbor size: %d\n", maze_.GetNeighbors(curr->x_, curr->y_).size());
    for (auto neighbor: maze_.GetNeighbors(curr->x_, curr->y_)) {
      if (!inPath(tmppath, neighbor)) {
        std::vector<Cell*> newpath = tmppath;
        newpath.push_back(neighbor);
        cells_to_visit.push(newpath);
      }
    }
  }

  curr_x_ = 0;
  curr_y_ = 0;
  shortestPath = possiblePaths[0];
  int minTurns = INT_MAX;
  for (auto a : possiblePaths) {
    Direction prevDir = Direction::NONE;
    int turns = 0;
    for (int i = 0; i < a.size() - 1; i++) {
      if (prevDir != GetDirection(a[i], a[i + 1])) {
        turns+=3;
      } else {
        turns+=1;
      }
    }
    if (turns < minTurns) {
      minTurns = turns;
      shortestPath = a;
    }
  }
  Log("Done computing the shortest path. At location: ");

  return shortestPath;
}

Direction Robot::GetDirection(Cell* start, Cell* end) {
  if (start->x_ == end->x_ && start->y_ + 1 == end->y_) {
    //printf("returning south\n");
    return Direction::NORTH;
  } else if (start->x_ == end->x_ && start->y_ == end->y_ + 1) {
    //printf("returning north\n");
    return Direction::SOUTH;
  } else if (start->x_ + 1 == end->x_ && start->y_ == end->y_) {
    //printf("returning east\n");
    return Direction::EAST;
  } else if (start->x_ == end->x_ + 1 && start->y_ == end->y_) {
    //printf("returning west\n");
    return Direction::WEST;
  }
  //printf("wtf? %d %d %d %d\n", start->x_, start->y_, end->x_, end->y_);
  return Direction::NONE;
}

void Robot::Run(std::vector<Cell*> path) {
  Log("Running the robot :)");
  Reset(false);
  for (int i = 0; i < path.size() - 1; i++) {
	if(robot_->readButton2())
	{
		Reset(false);
		return;
	}

    Direction dir = GetDirection(path[i], path[i + 1]);
    printf(" -> ");
    Move(GetDirection(path[i], path[i + 1]));
  }
}

void Robot::Log(const std::string& log) {
  if (enable_debugging_) {
    std::cout << log << std::endl;
  }
}

bool Robot::VisitCurrentCell() {
  Cell& cell = maze_.get(curr_x_, curr_y_);
  Cell* aboveCell;
  Cell* belowCell;
  Cell* leftCell;
  Cell* rightCell;
  Cell fakeCell;
  Cell* fake = &fakeCell;
  if (curr_y_ - 1 >= 0) {
    belowCell = &maze_.get(curr_x_, curr_y_ - 1);
  } else {
    belowCell = fake;
  }
  if (curr_y_ + 1 < maze_.rows()) {
    aboveCell = &maze_.get(curr_x_, curr_y_ + 1);
  } else {
    aboveCell = fake;
  }
  if (curr_x_ - 1 >= 0) {
    leftCell = &maze_.get(curr_x_ - 1, curr_y_);
  } else {
    leftCell = fake;
  }
  if (curr_x_ + 1 < maze_.cols()) {
    rightCell = &maze_.get(curr_x_ + 1, curr_y_);
  } else {
    rightCell = fake;
  }
  switch(orientation_) {
    case Direction::NORTH:
      cell.has_top_ = robot_->checkWallFrontClose();
      aboveCell->has_bottom_ = robot_->checkWallFrontClose();
      cell.has_left_ = robot_->checkWallLeft();
      leftCell->has_right_ = robot_->checkWallLeft();
      cell.has_right_ = robot_->checkWallRight();
      rightCell->has_left_ = robot_->checkWallRight();
    break;
    case Direction::SOUTH:
      cell.has_bottom_ = robot_->checkWallFrontClose();
      belowCell->has_top_ = robot_->checkWallFrontClose();
      cell.has_right_ = robot_->checkWallLeft();
      rightCell->has_left_ = robot_->checkWallLeft();
      cell.has_left_ = robot_->checkWallRight();
      leftCell->has_right_ = robot_->checkWallRight();
	  //printf("bot: %d right: %d left: %d\n", cell.has_bottom_, cell.has_right_, cell.has_left_);
    break;
    case Direction::WEST:
      cell.has_left_ = robot_->checkWallFrontClose();
      leftCell->has_right_ = robot_->checkWallFrontClose();
      cell.has_bottom_ = robot_->checkWallLeft();
      belowCell->has_top_ = robot_->checkWallLeft();
      cell.has_top_ = robot_->checkWallRight();
      aboveCell->has_bottom_ = robot_->checkWallRight();
    break;
    case Direction::EAST:
      cell.has_right_ = robot_->checkWallFrontClose();
      rightCell->has_left_ = robot_->checkWallFrontClose();
      cell.has_top_ = robot_->checkWallLeft();
      aboveCell->has_bottom_ = robot_->checkWallLeft();
      cell.has_bottom_ = robot_->checkWallRight();
      belowCell->has_top_ = robot_->checkWallRight();
    break;
  }
  return true;
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
  robot_->pid_drive();
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
  robot_->turn(degrees / 90, TURN_SPEED);
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

void Robot::moveToPath(std::vector<Cell*> tmppath) {
  Cell* curr = &maze_.get(curr_x_, curr_y_);
  Cell* start = curr;
  Cell* end = tmppath[tmppath.size() - 1];
  std::stack<Cell*> cells_to_visit;
  std::vector<Direction> path;

  Cell a = maze_.get(4, 2);
  Cell b = maze_.get(4, 3);

  printf("4 3 bot: %d", b.has_bottom_);
  printf("4 2 top: %d", a.has_top_);

  printf("moveToPath calculations\n");
  while (curr->x_ != end->x_ || curr->y_ != end->y_) {
  //for (int i = 0; i < 1; i++) {
    std::vector<Cell*> neighbors = maze_.GetNeighbors(curr->x_, curr->y_);
    printf("neighbors size: %d\n", neighbors.size());
    if (!curr->visited_) {
      for (Cell* neighbor : neighbors) {
        if (!neighbor->visited_) {
          printf("parent of %d %d is %d %d: \n", neighbor->x_, neighbor->y_, curr->x_, curr->y_);
          neighbor->parent_ = curr;
          cells_to_visit.push(neighbor);
        }
      }
    }
    curr->visited_ = true;
    printf("in stack: ");
    printf("x: %d y: %d, ", cells_to_visit.top()->x_, cells_to_visit.top()->y_);
    printf("\n");
    curr = cells_to_visit.top();
    cells_to_visit.pop();
    printf("setting curr to %d, %d\n", curr->x_, curr->y_);
  }

  while (curr->x_ != start->x_ || curr->y_ != start->y_) {
    path.insert(path.begin(), GetDirection(curr->parent_, curr));
    curr = curr->parent_;
  }

  for (auto d : path) {
    std::cout << "dir " << d << ", ";
    Move(d);
  }

  maze_.ClearVisitedAndParent();
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
