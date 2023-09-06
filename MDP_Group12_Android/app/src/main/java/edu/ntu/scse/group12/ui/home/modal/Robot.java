package edu.ntu.scse.group12.ui.home.modal;

public class Robot {
    private int row;
    private int col;
    private int direction;

    public static final int NORTH = 0;
    public static final int EAST = 2;
    public static final int SOUTH = 4;
    public static final int WEST = 6;

    public Robot(int row, int col, int direction) {
        this.row = row;
        this.col = col;
        this.direction = direction;
    }

    public int getRow() {
        return row;
    }
    public void setRow(int row) {
        this.row = row;
    }
    public int getCol() {
        return col;
    }
    public void setCol(int col) {
        this.col = col;
    }
    public int getDirection() {
        return direction;
    }

    public void setDirection(int direction) {
        this.direction = direction;
    }

    public void rotate() {
        this.direction = (this.direction + 2) % 8;
    }


    @Override
    public String toString() {
        return "Robot{" +
                "row=" + row +
                ", col=" + col +
                ", direction=" + direction +
                '}';
    }
}

