#ifndef MICROMOUSE_2018_ALGO_MAZE_H
#define MICROMOUSE_2018_ALGO_MAZE_H

#include <vector>
#include <string>
#include "Cell.h"

namespace algorithm {

class Maze {
 public:
  Maze(bool enable_debugging_, int rows, int cols);

  void Clear();

  void ClearVisitedAndParent();

  void print();

  void ReadFromFile();

  Cell& get(int x, int y);

  // Returns all of the neighbors of the given cell.
  std::vector<Cell*> GetNeighbors(int x, int y);

  // Returns the number of rows in the maze.
  int rows();

  // Returns the number of cols in the maze.
  int cols();

 private:
  bool enable_debugging_;
  int rows_;
  int cols_;
  std::vector<std::vector<Cell>> cells_;

  void Log(const std::string& log);
};

}

#endif //MICROMOUSE_2018_ALGO_MAZE_H
