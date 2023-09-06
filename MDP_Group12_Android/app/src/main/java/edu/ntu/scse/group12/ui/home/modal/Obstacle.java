package edu.ntu.scse.group12.ui.home.modal;

public class Obstacle {
    private int row;
    private int col;
    public boolean isBeingDragged;
    private int direction;

    private int imageId;
    private int id;
    public static final int NORTH = 0;
    public static final int EAST = 2;
    public static final int SOUTH = 4;
    public static final int WEST = 6;

    public Obstacle(int row, int col, int id, int direction) {
        this.row = row;
        this.col = col;
        this.id = id+1;
        this.direction = direction;
        this.imageId=0;
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

    public int getImageId() {
        return imageId;
    }
    public void setImageId(int imageId) {
        this.imageId = imageId;
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public void rotate() {
        this.direction = (this.direction + 2) % 8;
    }
    @Override
    public String toString() {
        return "Obstacle{" +
                "row=" + row +
                ", col=" + col +
                ", direction=" + direction +
                ", imageId=" + imageId +
                ", id=" + id +
                '}';
    }
}
