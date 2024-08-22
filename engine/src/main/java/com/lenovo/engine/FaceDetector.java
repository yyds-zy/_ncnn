package com.lenovo.engine;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import com.lenovo.engine.bean.FaceBox;

import java.util.List;

public class FaceDetector extends Component {

    private long nativeHandler;

    public FaceDetector() {
        nativeHandler = createInstance();
    }

    @Override
    public long createInstance() {
        return allocate();
    }

    public int loadModel(AssetManager assetsManager) {
        return nativeLoadModel(assetsManager);
    }

    public List<FaceBox> detect(Bitmap bitmap) {
        if (bitmap.getConfig() == Bitmap.Config.ARGB_8888) {
            return nativeDetectBitmap(bitmap);
        } else {
            throw new IllegalArgumentException("Invalid bitmap config value");
        }
    }

    public List<FaceBox> detect(byte[] yuv, int previewWidth, int previewHeight, int orientation) {
        if (previewWidth * previewHeight * 3 / 2 != yuv.length) {
            throw new IllegalArgumentException("Invalid yuv data");
        }
        return nativeDetectYuv(yuv, previewWidth, previewHeight, orientation);
    }

    @Override
    public void destroy() {
        deallocate();
    }

    // Native methods
    private native long allocate();

    private native void deallocate();

    private native int nativeLoadModel(AssetManager assetsManager);

    private native List<FaceBox> nativeDetectBitmap(Bitmap bitmap);

    private native List<FaceBox> nativeDetectYuv(byte[] yuv, int previewWidth, int previewHeight, int orientation);
}