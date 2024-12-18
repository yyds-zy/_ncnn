package com.lenovo.engine;

import android.content.res.AssetManager;
import android.graphics.Bitmap;

import com.lenovo.engine.bean.FaceBox;
import com.lenovo.engine.bean.ModelConfig;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Create by xuezhiyuan on 2024/12/3
 */
public class KeyPointDetector extends Component {
    private long nativeHandler;

    public KeyPointDetector() {
        nativeHandler = createInstance();
    }

    @Override
    public long createInstance() {
        return allocate();
    }

    public int loadModel(AssetManager assetsManager) {
        List<ModelConfig> configs = parseConfig(assetsManager);

        if (configs.isEmpty()) {
            System.err.println("parse model config failed");
            return -1;
        }

        return nativeLoadModel(assetsManager, configs);
    }

    private List<ModelConfig> parseConfig(AssetManager assetManager) {
        try (InputStream inputStream = assetManager.open("live/config.json");
             BufferedReader br = new BufferedReader(new InputStreamReader(inputStream))) {
            String line = br.readLine();
            JSONArray jsonArray = new JSONArray(line);

            List<ModelConfig> list = new ArrayList<>();
            for (int i = 0; i < jsonArray.length(); i++) {
                JSONObject config = jsonArray.getJSONObject(i);
                ModelConfig modelConfig = new ModelConfig();
                modelConfig.setName(config.optString("name"));
                modelConfig.setWidth(config.optInt("width"));
                modelConfig.setHeight(config.optInt("height"));
                modelConfig.setScale((float) config.optDouble("scale"));
                modelConfig.setShiftX((float) config.optDouble("shift_x"));
                modelConfig.setShiftY((float) config.optDouble("shift_y"));
                modelConfig.setOrgResize(config.optBoolean("org_resize"));

                list.add(modelConfig);
            }
            return list;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
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

    private native int nativeLoadModel(AssetManager assetsManager, List<ModelConfig> configs);

    private native List<FaceBox> nativeDetectBitmap(Bitmap bitmap);

    private native List<FaceBox> nativeDetectYuv(byte[] yuv, int previewWidth, int previewHeight, int orientation);
}
