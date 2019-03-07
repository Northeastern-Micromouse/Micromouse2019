#include "../include/Maze.h"

#include <cassert>
#include <iostream>
#include <fstream>

namespace algorithm {

using algorithm::Cell;

Maze::Maze(bool enable_debugging_, int rows, int cols)
  : enable_debugging_(enable_debugging_),
    rows_(rows),
    cols_(cols) {
  for (int ii = 0; ii < rows; ii++) {
    cells_.push_back(std::vector<Cell>(cols));
    for (int jj = 0; jj < cols; jj++) {
      cells_[ii][jj].Configure(jj, ii);
    }
  }
  /*
  std::ifstream myfile("maze.txt");

  std::string buf;
  std::string space("space");

  for (int jj = 0; jj < rows_; jj++) {
    for (int ii = 0; ii < cols_; ii++) {
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_top_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_bottom_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_left_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_right_ = false;
    }
  }
*/
  //this->print();

  assert(cells_.size() == rows_ && cells_[0].size() == cols_);
}

void Maze::Clear() {
  // No op.
}

void Maze::ClearVisitedAndParent(){
  for (auto& v : cells_) {
    for (auto& cell : v) {
      cell.visited_ = false;
      cell.parent_ = nullptr;
    }
  }
}

void Maze::print() {
  for (int jj = 0; jj < rows_; jj++) {
    for (int ii = 0; ii < cols_; ii++) {
      std::cout << "#";
      if (cells_[jj][ii].has_top_ == true)
        std::cout << "#";
      else
        std::cout << " ";
    }
      std::cout << "#";
    std::cout << std::endl;
    for (int ii = 0; ii < cols_; ii++) {
      if (cells_[jj][ii].has_left_ == true)
        std::cout << "#";
      else
        std::cout << " ";
      std::cout << " ";
    }
    if (cells_[jj][cols_ - 1].has_right_ == true)
        std::cout << "#";
    std::cout << std::endl;
  }

  for (int ii = 0; ii < cols_; ii++) {
    std::cout << "#";
    if (cells_[rows_ - 1][ii].has_bottom_ == true)
      std::cout << "#";
    else {
      std::cout << " ";
    }
  }
  std::cout << "#" << std::endl;
}

int Maze::rows() {
  return rows_;
}

int Maze::cols() {
  return cols_;
}

void Maze::ReadFromFile() {
  std::ifstream myfile("maze.txt");

  std::string buf;
  std::string space("space");

  for (int jj = 0; jj < rows_; jj++) {
    for (int ii = 0; ii < cols_; ii++) {
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_top_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_bottom_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_left_ = false;
        myfile >> buf;
      if (buf == space)
        cells_[ii][jj].has_right_ = false;
    }
  }
}

std::vector<Cell*> Maze::GetNeighbors(int x, int y) {
  std::vector<Cell*> ret;
  Cell cell = cells_[y][x];
  //Log("getting neighbors for row: " + std::to_string(y)
  //    + " col: " + std::to_string(x));

  if (y + 1 < rows_ && !cell.has_top_) {
    printf("has north\n");
    ret.push_back(&cells_[y + 1][x]);
  }
  if (y - 1 >= 0 && !cell.has_bottom_) {
    printf("has south \n");
    ret.push_back(&cells_[y - 1][x]);
  }
  if (x - 1 >= 0 && !cell.has_left_) {
    printf("has west\n");
    ret.push_back(&cells_[y][x - 1]);
  }
  if (x + 1 < cols_ && !cell.has_right_) {
    printf("has east\n");
    ret.push_back(&cells_[y][x + 1]);
  }
  return ret;
}

Cell& Maze::get(int x, int y) {
  return cells_[y][x];
}

void Maze::Log(const std::string& log) {
  if (enable_debugging_) {
    std::cout << log << std::endl;
  }
}

}
