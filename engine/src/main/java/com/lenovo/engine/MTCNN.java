package com.lenovo.engine;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

public class MTCNN extends Component {

    private long nativeHandler;

    public MTCNN() {
        nativeHandler = createInstance();
    }

    @Override
    public long createInstance() {
        return allocate();
    }

    public void loadModel(AssetManager assetManager) {
        nativeLoadModel(assetManager);
    }

    public void detect(Bitmap bitmap) {
        if (bitmap.getConfig() == Bitmap.Config.ARGB_8888) {
            nativeDetectBitmap(bitmap);
        } else {
            throw new IllegalArgumentException("Invalid bitmap config value");
        }
    }

    @Override
    public void destroy() {
        deallocate();
    }

    // Native methods
    private native long allocate();

    private native void deallocate();

    private native void nativeLoadModel(AssetManager assetManager);

    private native void nativeDetectBitmap(Bitmap bitmap);
}
