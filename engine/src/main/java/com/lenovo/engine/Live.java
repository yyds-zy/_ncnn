package com.lenovo.engine;

import android.content.res.AssetManager;

import com.lenovo.engine.bean.FaceBox;
import com.lenovo.engine.bean.ModelConfig;

import org.json.JSONArray;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

public class Live extends Component {
    private long nativeHandler;

    public Live() {
        nativeHandler = createInstance();
    }

    public long createInstance() {
        return allocate();
    }

    public void destroy() {
        deallocate();
    }

    public int loadModel(AssetManager assetManager) {
        List<ModelConfig> configs = parseConfig(assetManager);

        if (configs.isEmpty()) {
            System.err.println("parse model config failed");
            return -1;
        }

        return nativeLoadModel(assetManager, configs);
    }

    public float detect(byte[] yuv, int previewWidth, int previewHeight, int orientation, FaceBox faceBox) {
        if (previewWidth * previewHeight * 3 / 2 != yuv.length) {
            throw new IllegalArgumentException("Invalid yuv data");
        }

        return nativeDetectYuv(yuv, previewWidth, previewHeight, orientation, faceBox.getLeft(), faceBox.getTop(), faceBox.getRight(), faceBox.getBottom());
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

    // Native methods
    private native long allocate();

    private native void deallocate();

    private native int nativeLoadModel(AssetManager assetManager, List<ModelConfig> configs);

    private native float nativeDetectYuv(byte[] yuv, int previewWidth, int previewHeight, int orientation, int left, int top, int right, int bottom);
}
