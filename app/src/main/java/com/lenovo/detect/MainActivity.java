package com.lenovo.detect;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.PixelFormat;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.media.Image;
import android.media.ImageReader;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.provider.Settings;
import android.util.Log;
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import com.lenovo.engine.bean.FaceBox;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.List;

/**
 * https://blog.csdn.net/tong5956/article/details/82688886
 */
public class MainActivity extends AppCompatActivity {

    public static final String TAG = "MainActivity";
    private static final int PERMISSIONS_REQUEST_CODE = 1;
    private static final int SIZE_WIDTH = 640 ;
    private static final int SIZE_HEIGHT = 480 ;
    private SurfaceView surfaceView;
    private CameraManager mCameraManager;
    private Handler childHandler, mainHandler;
    private CameraCaptureSession mCameraCaptureSession;
    private CameraDevice mCameraDevice;
    private TextView mFaceTv;
    private float Threshold = 0.915F;
    private Switch mControlDetect;
    private boolean isFaceDetect;
    private SurfaceHolder mSurfaceHolder;
    private RectView rectView;


    @SuppressLint("MissingInflatedId")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mFaceTv = findViewById(R.id.face_tv);
        rectView = findViewById(R.id.rect);
        mControlDetect = findViewById(R.id.switch_detect);

        checkAndRequestPermissions();

        boolean init = EngineWrapper.getInstance().init(this);
        if (init) {
            Toast.makeText(this, "Engine init success.", Toast.LENGTH_LONG).show();
        } else {
            Toast.makeText(this, "Engine init failed.", Toast.LENGTH_LONG).show();
            finish();
        }

//        List<FaceBox> faceBoxes = EngineWrapper.getInstance().detectFace(loadBitmapFromAsset(getApplicationContext(), "img.png"));
//        boolean empty = faceBoxes.isEmpty();
//        Log.d("MainActivity", empty + "");

        if (!checkPermission()) {
            requestPermission();
        }


        surfaceView = findViewById(R.id.surfaceView);
        // 获得 SurfaceHolder 对象
        mSurfaceHolder = surfaceView.getHolder();
        //判断是前置还是后置
        surfaceView.getLayoutParams().width = SIZE_WIDTH;
        surfaceView.getLayoutParams().height = SIZE_HEIGHT;
        // 设置 Surface 格式
        // 参数： PixelFormat中定义的 int 值 ,详细参见 PixelFormat.java
        mSurfaceHolder.setFormat(PixelFormat.TRANSPARENT);
        mSurfaceHolder.setKeepScreenOn(true);
        mSurfaceHolder.addCallback(mSurfaceCallBack);

        mControlDetect.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                // 打开
                isFaceDetect = true;
                rectView.setVisibility(View.VISIBLE);
//                MMKVManager.getInstance().set("is_face_detect", true);
                Bitmap bitmap = loadBitmapFromAsset(getBaseContext(), "121.png");
                if (bitmap != null) {
                    EngineWrapper.getInstance().detectAge(bitmap);
                }

            } else {
                // 关闭
                isFaceDetect = false;
                rectView.setVisibility(View.GONE);
//                MMKVManager.getInstance().set("is_face_detect", false);

                Bitmap bitmap = loadBitmapFromAsset(getBaseContext(), "imgs.png");
                if (bitmap != null) {
                    EngineWrapper.getInstance().detectAge(bitmap);
                }
            }
        });
        mControlDetect.setChecked(isFaceDetect);
    }
    private ImageReader mImageReaderPreview;
    private SurfaceHolder.Callback mSurfaceCallBack = new SurfaceHolder.Callback() {

        @Override
        public void surfaceCreated(@NonNull SurfaceHolder holder) {
            HandlerThread handlerThread = new HandlerThread("Camera2");
            handlerThread.start();
            childHandler = new Handler(handlerThread.getLooper());
            mainHandler = new Handler(getMainLooper());
            mImageReaderPreview = ImageReader.newInstance(SIZE_WIDTH, SIZE_HEIGHT, ImageFormat.YUV_420_888,1);
            mImageReaderPreview.setOnImageAvailableListener(new ImageReader.OnImageAvailableListener() { //可以在这里处理拍照得到的临时照片 例如，写入本地
                @Override
                public void onImageAvailable(ImageReader reader) {
                    Image image = reader.acquireLatestImage();
                    final byte[] data = ImageUtil.getBytesFromImageAsType(image, ImageUtil.NV21);
                    DetectionResult detect = EngineWrapper.getInstance().detect(data, SIZE_WIDTH, SIZE_HEIGHT, 1);
                    boolean hasFace = detect.isHasFace();
                    runOnUiThread(() -> {
                        if (hasFace) {
                            //left：639 top:410 right:890 bottom:661
                            if (mFaceTv.getVisibility() == View.GONE) {
                                mFaceTv.setVisibility(View.VISIBLE);
                            }
                            if (rectView.getVisibility() == View.GONE && isFaceDetect) {
                                rectView.setVisibility(View.VISIBLE);
                            }
                            int left = detect.getLeft();
                            int top = detect.getTop();
                            int right = detect.getRight();
                            int bottom = detect.getBottom();
                            float confidence = detect.getConfidence();

                            if (confidence >= Threshold) {
                                mFaceTv.setText("真脸");
                            } else {
                                mFaceTv.setText("假脸");
                            }

                            if (confidence >= Threshold) {
                                rectView.setX1(SIZE_WIDTH - left);
                                rectView.setX2(SIZE_WIDTH - right);
                                rectView.setY1(top);
                                rectView.setY2(bottom);
                                rectView.setConfidence(confidence);
                                rectView.invalidate();
                            }
                        } else {
                            mFaceTv.setVisibility(View.GONE);
                            rectView.setVisibility(View.GONE);
                        }
                    });
                    image.close();
                }
            }, null);
            //获取摄像头管理
            mCameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
            try {
                if (ActivityCompat.checkSelfPermission(MainActivity.this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
                    return;
                }
                //打开摄像头
                mCameraManager.openCamera(String.valueOf(1), stateCallback, mainHandler);
            } catch (CameraAccessException e) {
                e.printStackTrace();
            }

        }

        @Override
        public void surfaceChanged(@NonNull SurfaceHolder holder, int format, int width, int height) {

        }

        @Override
        public void surfaceDestroyed(@NonNull SurfaceHolder holder) {

        }
    };

    private void takePreview() {
        try {
            // 创建预览需要的CaptureRequest.Builder
            final CaptureRequest.Builder previewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
            // 将SurfaceView的surface作为CaptureRequest.Builder的目标
            previewRequestBuilder.addTarget(mSurfaceHolder.getSurface());
            previewRequestBuilder.addTarget(mImageReaderPreview.getSurface());
            // 创建CameraCaptureSession，该对象负责管理处理预览请求和拍照请求
            mCameraDevice.createCaptureSession(Arrays.asList(mSurfaceHolder.getSurface(), mImageReaderPreview.getSurface()), new CameraCaptureSession.StateCallback() // ③
            {
                @Override
                public void onConfigured(CameraCaptureSession cameraCaptureSession) {
                    if (null == mCameraDevice) return;
                    // 当摄像头已经准备好时，开始显示预览
                    mCameraCaptureSession = cameraCaptureSession;
                    try {
                        // 自动对焦
                        previewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
                        // 打开闪光灯
                        previewRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON_AUTO_FLASH);
                        // 显示预览
                        CaptureRequest previewRequest = previewRequestBuilder.build();
                        mCameraCaptureSession.setRepeatingRequest(previewRequest, null, childHandler);
                    } catch (CameraAccessException e) {
                        e.printStackTrace();
                    }
                }

                @Override
                public void onConfigureFailed(CameraCaptureSession cameraCaptureSession) {
                    Toast.makeText(MainActivity.this, "配置失败", Toast.LENGTH_SHORT).show();
                }
            }, childHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
        @Override
        public void onOpened(CameraDevice camera) {//打开摄像头
            mCameraDevice = camera;
            //开启预览
            takePreview();
        }

        @Override
        public void onDisconnected(CameraDevice camera) {//关闭摄像头
            if (null != mCameraDevice) {
                mCameraDevice.close();
                MainActivity.this.mCameraDevice = null;
            }
        }

        @Override
        public void onError(CameraDevice camera, int error) {//发生错误
            Toast.makeText(MainActivity.this, "摄像头开启失败" + error, Toast.LENGTH_SHORT).show();
        }
    };


    @Override
    protected void onDestroy() {
        super.onDestroy();
        EngineWrapper.getInstance().unInit();
    }

    private boolean checkPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
    }

    private void requestPermission() {
        ActivityCompat.requestPermissions(this,
                new String[]{Manifest.permission.CAMERA},
                PERMISSIONS_REQUEST_CODE);
    }

    private void checkAndRequestPermissions() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED ||
                ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.READ_EXTERNAL_STORAGE, Manifest.permission.WRITE_EXTERNAL_STORAGE}, 203);
        } else {
            Toast.makeText(this,"已经有了文件权限",Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == PERMISSIONS_REQUEST_CODE) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // 开启
                takePreview();
            } else {
                finish();
            }
        }
    }

    public Bitmap loadBitmapFromAsset(Context context, String imagePath) {
        // 获取AssetManager实例
        AssetManager assetManager = context.getAssets();

        // 使用AssetManager打开图片文件
        InputStream is = null;
        try {
            is = assetManager.open(imagePath);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // 使用BitmapFactory.decodeStream来将InputStream转换成Bitmap
        Bitmap bitmap = null;
        if (is != null) {
            bitmap = BitmapFactory.decodeStream(is);
            try {
                is.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        return bitmap;
    }
}