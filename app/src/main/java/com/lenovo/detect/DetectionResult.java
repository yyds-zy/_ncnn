package com.lenovo.detect;

public class DetectionResult {
    private int left;
    private int top;
    private int right;
    private int bottom;
    private float confidence;
    private long time;
    private boolean hasFace;

    public DetectionResult(){}

    public DetectionResult(int left, int top, int right, int bottom, float confidence, long time, boolean hasFace) {
        this.left = left;
        this.top = top;
        this.right = right;
        this.bottom = bottom;
        this.confidence = confidence;
        this.time = time;
        this.hasFace = hasFace;
    }

    public int getLeft() {
        return left;
    }

    public void setLeft(int left) {
        this.left = left;
    }

    public int getTop() {
        return top;
    }

    public void setTop(int top) {
        this.top = top;
    }

    public int getRight() {
        return right;
    }

    public void setRight(int right) {
        this.right = right;
    }

    public int getBottom() {
        return bottom;
    }

    public void setBottom(int bottom) {
        this.bottom = bottom;
    }

    public float getConfidence() {
        return confidence;
    }

    public void setConfidence(float confidence) {
        this.confidence = confidence;
    }

    public long getTime() {
        return time;
    }

    public void setTime(long time) {
        this.time = time;
    }

    public boolean isHasFace() {
        return hasFace;
    }

    public void setHasFace(boolean hasFace) {
        this.hasFace = hasFace;
    }
}
