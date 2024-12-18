package com.lenovo.engine;

/**
 * Create by xuezhiyuan on 2024/12/3
 */

import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.util.Log;

public class AgeDetector extends Component {

    private long nativeHandler;

    public AgeDetector() {
        nativeHandler = createInstance();
    }

    @Override
    public long createInstance() {
        return allocate();
    }

    public int loadModel(AssetManager assetManager) {
        return nativeLoadModel(assetManager);
    }

    public void detect(Bitmap bitmap) {
        if (bitmap == null) {
        //if (bitmap.getConfig() == Bitmap.Config.ARGB_8888) {
            long start_1 = System.currentTimeMillis();
            int age = nativeDetectAge(bitmap);
            Log.d("AgeDetector", "检测年龄：" + age);
            long end_1 = System.currentTimeMillis();
            long time_1 = end_1 - start_1;
            Log.d("xuezhiyuan", "年龄检测耗时：" + time_1 + " ms");

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

    private native int nativeLoadModel(AssetManager assetManager);

    private native int nativeDetectAge(Bitmap bitmap);
}
