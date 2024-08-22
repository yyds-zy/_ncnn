package com.lenovo.detect;

import android.content.Context;
import android.graphics.Bitmap;
import android.os.Environment;

import com.lenovo.engine.MTCNN;
import com.lenovo.engine.FaceDetector;
import com.lenovo.engine.Live;
import com.lenovo.engine.bean.FaceBox;

import java.util.List;

public class EngineWrapper {
    public static String modelFile = Environment.getExternalStorageDirectory().getPath() + "/lenovo_model/";

    private Live live = new Live();
    private FaceDetector faceDetector = new FaceDetector();
    private MTCNN ageDetector = new MTCNN();
    private static EngineWrapper instance;
    private EngineWrapper() {}

    public static EngineWrapper getInstance() {
        if (instance == null) {
            synchronized (EngineWrapper.class) {
                if (instance == null) {
                    instance = new EngineWrapper();
                }
            }
        }
        return instance;
    }

    /**
     * 初始化模型
     * @param context
     * @return
     */
    public boolean init(Context context) {
        int ret = faceDetector.loadModel(context.getAssets());
        if (ret == 0) {
            ret = live.loadModel(context.getAssets());
            if (ret == 0) {
                ageDetector.loadModel(context.getAssets());
                return true;
            }
        }
        return false;
    }

    public void unInit() {
        faceDetector.destroy();
        live.destroy();
    }

    public List<FaceBox> detectFace(Bitmap bitmap) {
        List<FaceBox> ret = faceDetector.detect(bitmap);
        return ret;
    }

    public List<FaceBox> detectFace(byte[] yuv, int width, int height, int ori) {
        List<FaceBox> ret = faceDetector.detect(yuv, width, height, ori);
        return ret;
    }

    public float detectLive(byte[] yuv, int width, int height, int ori, FaceBox faceBox) {
        float ret = live.detect(yuv, width, height, ori, faceBox);
        return ret;
    }

    public void detectAge(Bitmap bitmap) {
        ageDetector.detect(bitmap);
    }

    public DetectionResult detect(byte[] yuv, int width, int height, int ori) {
        List<FaceBox> faceBoxes = detectFace(yuv, width, height, ori);
        if (!faceBoxes.isEmpty()) {
            long start = System.currentTimeMillis();
            FaceBox faceBox = faceBoxes.get(0);
            float c = detectLive(yuv, width, height, ori, faceBox);
            faceBox.setConfidence(c);

            long end = System.currentTimeMillis();
            long time = end - start;
            return new DetectionResult(faceBox.getLeft(), faceBox.getTop(), faceBox.getRight(), faceBox.getBottom(),
                    faceBox.getConfidence(), time, true);
        }
        return new DetectionResult();
    }
}
