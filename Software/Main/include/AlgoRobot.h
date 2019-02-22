#ifndef MICROMOUSE_ALGO_ROBOT_H
#define MICROMOUSE_ALGO_ROBOT_H

#include <stack>
#include <string>

#include "Direction.h"
#include "Maze.h"
#include "robot.h"

namespace algorithm {

// If the robot is at cell 0,0 then North is assumed to be cell 0,1, east is assumed to be cell 1,0, west is assumed
// to be cell -1, 0 and south is assumed to be cell 0, -1.
class Robot {
 public:
  struct CellPair {
    CellPair(Cell* first, Cell second)
        : first_(first),
          second_(second) {
      // No op.
    }

    Cell* first_;
    Cell second_;
  };
  Robot(bool enable_debugging_, int maze_x, int maze_y, algorithm::Direction orientation);

  // Maps the given maze. Does not return until the entire maze has been mapped.
  void Map();

  // Can only be called after a call to Map. Computes the shortest path to the given goal.
  void ComputeShortestPath();

  // Can only be called after ComputeShortestPath. Drives the robot to the goal.
  void Run();
 private:
  micromouse::Robot winslow_;
  algorithm::Maze maze_;
  std::stack<CellPair> neighbors_;
  algorithm::Direction orientation_;
  std::vector<Direction> path_;
  bool enable_debugging_;
  int curr_x_;
  int curr_y_;
  float driveSpeed_;
  float turnSpeed_;
  // The goal is represented by the box created with these 4 points.
  int top_left_goal_x_;
  int top_left_goal_y_;
  int bottom_right_goal_x_;
  int bottom_right_goal_y_;

  void Log(const std::string& log);
  // Returns true if any items were added to the neighbors.
  bool VisitCurrentCell();

  // Moves the robot one cell forward in the given direction. If the robot is not facing that direction it will
  // rotate the robot to face that direction and then move it forward. It will only return when the robot is done
  // driving to the given cell.
  void Move(algorithm::Direction dir);

  // Turns the robot so that it is facing north. This call does nothing if the robot is already facing north.
  void TurnNorth();
  // Turns the robot so that it is facing east. This call does nothing if the robot is already facing east.
  void TurnEast();
  // Turns the robot so that it is facing west. This call does nothing if the robot is already facing west.
  void TurnWest();
  // Turns the robot so that it is facing south. This call does nothing if the robot is already facing south.
  void TurnSouth();

  // Rotates the robot the given degrees.
  void Rotate(int degrees);

  // Moves the robot in the opposite direction.
  void GoBack(algorithm::Direction dir);

  // Returns the direction the robot would need to move in order to get to the desired cell. The cell should only be
  // only cell away from the current location of the robot.
  Direction GetDirection(Cell* cell);

  Direction GetDirection(Cell* start, Cell* end);

  std::vector<Direction> GetPath(Cell* start, Cell* end);

  // Returns true if the robot is inside the goal.
  bool IsInsideGoal();
  // Returns true if the two points are inside the goal.
  bool IsInsideGoal(int x, int y);

  void PrintPath(const std::vector<Direction>& path);
};

}

#endif //MICROMOUSE_ALGO_ROBOT_H
