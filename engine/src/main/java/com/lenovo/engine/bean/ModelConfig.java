package com.lenovo.engine.bean;

/**
 * Create by xuezhiyuan on 2024/12/3
 */

public class ModelConfig {
    private String name;
    private int width;
    private int height;
    private float scale;
    private float shift_x;
    private float shift_y;
    private boolean org_resize;

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getWidth() {
        return width;
    }

    public void setWidth(int width) {
        this.width = width;
    }

    public int getHeight() {
        return height;
    }

    public void setHeight(int height) {
        this.height = height;
    }

    public float getScale() {
        return scale;
    }

    public void setScale(float scale) {
        this.scale = scale;
    }

    public float getShiftX() {
        return shift_x;
    }

    public void setShiftX(float shift_x) {
        this.shift_x = shift_x;
    }

    public float getShiftY() {
        return shift_y;
    }

    public void setShiftY(float shift_y) {
        this.shift_y = shift_y;
    }

    public boolean isOrgResize() {
        return org_resize;
    }

    public void setOrgResize(boolean org_resize) {
        this.org_resize = org_resize;
    }
}
