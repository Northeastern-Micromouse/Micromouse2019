#ifndef MICROMOUSE_CELL_H
#define MICROMOUSE_CELL_H

namespace algorithm {

class Cell {
 public:
  Cell();

  Cell(int x, int y);

  void Configure(int x, int y);

  // Prints the cell and all of its values.
  void Print();

  // Public to avoid getters and setters
  int x_;
  int y_;
  bool visited_;
  bool mapped_;
  Cell* parent_;
  bool has_left_; // True iff there is a wall to the left of the cell.
  bool has_right_; // True iff there is a wall to the right of the cell.
  bool has_top_; // True iff there is a wall to the top of the cell.
  bool has_bottom_; // True iff there is a wall to the bottom of the cell.
};

}

#endif //MICROMOUSE_CELL_H
