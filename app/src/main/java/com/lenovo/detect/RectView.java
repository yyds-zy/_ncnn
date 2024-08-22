package com.lenovo.detect;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.*;
import android.util.AttributeSet;
import android.view.View;
import java.text.DecimalFormat;

public class RectView extends View {

    private float confidence;
    private RectF rect;
    private Paint paint;
    private int color;
    private float radius;
    private int textPadding;
    private DecimalFormat decimalFormat;
    private float lineLength;
    private RectF leftTopAcrRectF = new RectF();
    private RectF rightTopAcrRectF = new RectF();
    private RectF leftBottomAcrRectF = new RectF();
    private RectF rightBottomAcrRectF = new RectF();
    private Rect textBackgroundRect = new Rect();
    private Rect textBoundsRect = new Rect();
    private int textWidth;
    private int textHeight;

    private static final String SAMPLE_TEXT = "0.789";
    private static final int DEFAULT_LEFT = 0;
    private static final int DEFAULT_TOP = 0;
    private static final int DEFAULT_RIGHT = 0;
    private static final int DEFAULT_BOTTOM = 0;
    private static final float DEFAULT_TEXT_SIZE = 50;
    private static final int DEFAULT_TEXT_PADDING = 8;
    private static final float DEFAULT_RADIUS = 0;
    private static final float DEFAULT_LINE_LENGTH = 45;
    private static final int DEFAULT_COLOR = Color.RED;

    public RectView(Context context) {
        this(context, null);
    }

    public RectView(Context context, AttributeSet attrs) {
        this(context, attrs, 0);
    }

    public RectView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        final TypedArray typedArray = context.obtainStyledAttributes(attrs, R.styleable.RectView);

        int left = typedArray.getInt(R.styleable.RectView_x1, DEFAULT_LEFT);
        int top = typedArray.getInt(R.styleable.RectView_y1, DEFAULT_TOP);
        int right = typedArray.getInt(R.styleable.RectView_x2, DEFAULT_RIGHT);
        int bottom = typedArray.getInt(R.styleable.RectView_y2, DEFAULT_BOTTOM);
        color = typedArray.getColor(R.styleable.RectView_color, DEFAULT_COLOR);
        float textSize = typedArray.getDimension(R.styleable.RectView_textSize, DEFAULT_TEXT_SIZE);
        textPadding = (int) typedArray.getDimension(R.styleable.RectView_textPadding, DEFAULT_TEXT_PADDING);
        radius = typedArray.getDimension(R.styleable.RectView_radius, DEFAULT_RADIUS);
        lineLength = typedArray.getDimension(R.styleable.RectView_lineLength, DEFAULT_LINE_LENGTH);

        typedArray.recycle();

        rect = new RectF(left, top, right, bottom);

        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(3);
        paint.setTextSize(textSize);

        String sampleText = "0.789";
        paint.getTextBounds(sampleText, 0, sampleText.length(), textBoundsRect);
        textWidth = textBoundsRect.width();
        textHeight = textBoundsRect.height();

        decimalFormat = new DecimalFormat("0.000");
    }

    public void setX1(int v) {
        rect.left = v;
    }

    public void setX2(int v) {
        rect.right = v;
    }

    public void setY1(int v) {
        rect.top = v;
    }

    public void setY2(int v) {
        rect.bottom = v;
    }

    public void setColor(int v) {
        color = v;
        paint.setColor(color);
    }

    public void setConfidence(float v) {
        confidence = v;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(color);

        // left top arc
        canvas.drawLine(rect.left, rect.top + lineLength, rect.left, rect.top + radius, paint);
//        leftTopAcrRectF.set(rect.left, rect.top - radius, rect.left + radius * 2, rect.top + radius);
//        canvas.drawArc(leftTopAcrRectF, 180, 90, false, paint);
        canvas.drawLine(rect.left + radius, rect.top, rect.left - lineLength, rect.top, paint);

        // right top arc
        canvas.drawLine(rect.right + lineLength, rect.top, rect.right - radius, rect.top, paint);
//        rightTopAcrRectF.set(rect.right - radius * 2, rect.top - radius, rect.right, rect.top + radius);
//        canvas.drawArc(rightTopAcrRectF, 270, 90, false, paint);
        canvas.drawLine(rect.right, rect.top + radius, rect.right, rect.top + lineLength, paint);

        // left bottom arc
        canvas.drawLine(rect.left, rect.bottom - lineLength, rect.left, rect.bottom - radius, paint);
//        leftBottomAcrRectF.set(rect.left, rect.bottom - radius * 2, rect.left + radius * 2, rect.bottom);
//        canvas.drawArc(leftBottomAcrRectF, 180, -90, false, paint);
        canvas.drawLine(rect.left + radius, rect.bottom, rect.left - lineLength, rect.bottom, paint);

        // right bottom arc
        canvas.drawLine(rect.right + lineLength, rect.bottom, rect.right - radius, rect.bottom, paint);
//        rightBottomAcrRectF.set(rect.right - radius * 2, rect.bottom - radius * 2, rect.right, rect.bottom);
//        canvas.drawArc(rightBottomAcrRectF, 90, -90, false, paint);
        canvas.drawLine(rect.right, rect.bottom - radius, rect.right, rect.bottom - lineLength, paint);

        String text = decimalFormat.format(confidence);
        textBackgroundRect.set((int) rect.left, (int) (rect.top - textHeight - 2 * textPadding), (int) (rect.left + textWidth + 2 * textPadding), (int) (rect.top));
        paint.setStyle(Paint.Style.FILL);
        canvas.drawRect(textBackgroundRect, paint);

        textBoundsRect.set((int) (rect.left + textPadding), (int) (rect.top - textPadding - textHeight), (int) (rect.left + textPadding + textWidth), (int) (rect.top - textPadding));
        paint.setColor(Color.WHITE);
        canvas.drawText(text, textBoundsRect.left, textBoundsRect.bottom, paint);
    }
}