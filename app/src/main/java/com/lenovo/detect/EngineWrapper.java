package com.lenovo.detect;

import android.content.Context;
import android.graphics.Bitmap;
import android.util.Log;

import com.lenovo.engine.KeyPointDetector;
import com.lenovo.engine.bean.FaceBox;
import java.util.List;

public class EngineWrapper {

    private float LiveThreshold = 0.457F;
    //private Live live = new Live();
    //private AgeDetector ageDetector = new AgeDetector();
    private KeyPointDetector keyPointDetector = new KeyPointDetector();
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
        int ret = keyPointDetector.loadModel(context.getAssets());
        if (ret == 0) {
            return true;
        }
        return false;
    }

    public void unInit() {
//        live.destroy();
//        ageDetector.destroy();
        keyPointDetector.destroy();
    }

    public List<FaceBox> detectFaceTmp(byte[] yuv, int width, int height, int ori) {
        List<FaceBox> ret = keyPointDetector.detect(yuv, width, height, ori);
        return ret;
    }

//    public float detectLive(byte[] yuv, int width, int height, int ori, FaceBox faceBox) {
//        float ret = live.detect(yuv, width, height, ori, faceBox);
//        return ret;
//    }

    public DetectionResult detect(byte[] yuv, int width, int height, int ori) {
        long start_1 = System.currentTimeMillis();
        List<FaceBox> faceBoxes = detectFaceTmp(yuv, width, height, ori);
        long end_1 = System.currentTimeMillis();
        long time_1 = end_1 - start_1;
        Log.d("EngineWrapper", "总检测耗时：" + time_1 + " ms");

        if (!faceBoxes.isEmpty()) {
            long start = System.currentTimeMillis();
            FaceBox faceBox = faceBoxes.get(0);
            long end = System.currentTimeMillis();
            long time = end - start;
            if (faceBox.getConfidence() >= LiveThreshold) {
                return new DetectionResult(faceBox.getLeft(), faceBox.getTop(), faceBox.getRight(), faceBox.getBottom(),
                        faceBox.getConfidence(), time, true, faceBox.getAge());
            } else {
                return new DetectionResult(faceBox.getLeft(), faceBox.getTop(), faceBox.getRight(), faceBox.getBottom(),
                        faceBox.getConfidence(), time, false, 0);
            }
        }
        return new DetectionResult();
    }
}
