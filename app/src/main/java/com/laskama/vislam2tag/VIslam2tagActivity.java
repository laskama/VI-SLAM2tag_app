/*
 * Copyright 2018 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.laskama.vislam2tag;

import android.Manifest;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener2;
import android.hardware.SensorManager;
import android.media.Image;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.SystemClock;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.ar.core.ArCoreApk;
import com.google.ar.core.AugmentedImage;
import com.google.ar.core.AugmentedImageDatabase;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraConfig;
import com.google.ar.core.CameraConfigFilter;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Config;
import com.google.ar.core.Frame;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import com.google.ar.core.examples.java.TaskRunner;
import com.google.ar.core.examples.java.common.helpers.CameraPermissionHelper;
import com.google.ar.core.examples.java.common.helpers.FullScreenHelper;
import com.google.ar.core.examples.java.common.helpers.SnackbarHelper;
import com.google.ar.core.examples.java.common.helpers.TrackingStateHelper;
import com.google.ar.core.examples.java.computervision.CpuImageDisplayRotationHelper;
import com.google.ar.core.examples.java.computervision.CpuImageRenderer;
import com.google.ar.core.examples.java.computervision.EdgeDetector;
import com.google.ar.core.examples.java.computervision.FrameTimeHelper;
import com.laskama.vislam2tag.R;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/** This is a simple example that demonstrates CPU image access with ARCore. */
public class VIslam2tagActivity extends AppCompatActivity implements GLSurfaceView.Renderer, SensorEventListener2 {
  private static final String TAG = VIslam2tagActivity.class.getSimpleName();
  private static final String CAMERA_INTRINSICS_TEXT_FORMAT =
      "\tUnrotated Camera %s %s Intrinsics:\n\tFocal Length: (%.2f, %.2f)"
          + "\n\tPrincipal Point: (%.2f, %.2f)"
          + "\n\t%s Image Dimensions: (%d, %d)"
          + "\n\tUnrotated Field of View: (%.2f˚, %.2f˚)"
          + "\n\tRender frame time: %.1f ms (%.0ffps)"
          + "\n\tCPU image frame time: %.1f ms (%.0ffps)"
          + "\n\tPose: %s";
  private static final float RADIANS_TO_DEGREES = (float) (180 / Math.PI);

  //
  //  Computervision_java examples instance variables
  //

  // Session management and rendering.
  private GLSurfaceView surfaceView;
  private Session session;
  private Config config;
  private boolean installRequested;
  private final SnackbarHelper messageSnackbarHelper = new SnackbarHelper();
  private CpuImageDisplayRotationHelper cpuImageDisplayRotationHelper;
  private final TrackingStateHelper trackingStateHelper = new TrackingStateHelper(this);
  private final CpuImageRenderer cpuImageRenderer = new CpuImageRenderer();

  // This lock prevents changing resolution as the frame is being rendered. ARCore requires all
  // CPU images to be released before changing resolution.
  private final Object frameImageInUseLock = new Object();

  // Camera intrinsics text view.
  private TextView cameraIntrinsicsTextView;
  private TextView rssTextView;

  private final FrameTimeHelper renderFrameTimeHelper = new FrameTimeHelper();
  private final FrameTimeHelper cpuImageFrameTimeHelper = new FrameTimeHelper();

  private EdgeDetector edgeDetector = new EdgeDetector();

  //
  // VI-SLAM2tag instance variables
  //

  // Activation of control-point-based validation
  // if true, the app will show a button ("Control-Point"), which can
  // be pressed every time a control-point is visited. This will log the timestamp,
  // which can be used for assesing the labeling accuracy
  private boolean CP_VALIDATION_ENABLED = false;

  // sensor recordings
  private TaskRunner taskRunner;
  private SensorManager manager;
  private BroadcastReceiver wifiScanReceiver;
  private WifiManager wifiManager;

  // FileWriter for logging data (each data source has its own writer)
  private FileWriter writer;
  private FileWriter wifiWriter;
  private FileWriter poseWriter;
  private FileWriter initPoseWriter;
  private FileWriter refMarkerWriter;

  // Whether data logging is enabled
  private boolean writeFiles = true;

  // Local data structure for storing logged data until written to disk as batch job
  private List<String> sensorLines = new ArrayList<>();
  private List<String> poseLines = new ArrayList<>();
  private List<String> initPoseLines = new ArrayList<>();

  // List for storing the currently tracked Augmented images
  // those will be used for logging their poses for every received new camera frame
  private List<AugmentedImage> trackedImages = new ArrayList<>();

  // Button for registering whenever user passes reference marker (for evaluation purpose only)
  private Button markerButton;

  // counters for user feedback on how much data were already collected
  private int rssCounter = 0;
  private int markerCounter = 0;

  private List<Integer> markerIdx = new ArrayList<>();


  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    // connect the view with the activity (controller) and set listeners
    surfaceView = findViewById(R.id.surfaceview);
    cameraIntrinsicsTextView = findViewById(R.id.camera_intrinsics_view);
    rssTextView = findViewById(R.id.rssTextView);
    surfaceView = findViewById(R.id.surfaceview);
    markerButton = findViewById(R.id.markerButton);

    // Adapt the view of the app based on whether control-point validation is enabled
    if (CP_VALIDATION_ENABLED) {
      markerButton.setVisibility(View.VISIBLE);
      markerButton.setOnClickListener(v -> registerClickedMarker());
    } else {
      markerButton.setVisibility(View.INVISIBLE);
    }

    // handle permissions once the activity is created
    requestAppPermissions();

    // CPU rendering setup (from examples)
    setupCPUrendering();

    // Configure everything related to VI-SLAM2tag data recording
    // setup sensor manager that is used for registering listener to sensor events
    manager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

    // setup wifi manager that is used for requesting WiFi network scans
    wifiManager = (WifiManager)
            getApplicationContext().getSystemService(Context.WIFI_SERVICE);

    // setup the wifi broadcast receiver that is called whenever a WiFi scan is completed
    wifiScanReceiver = new BroadcastReceiver() {
      @Override
      public void onReceive(Context c, Intent intent) {
        boolean success = intent.getBooleanExtra(
                WifiManager.EXTRA_RESULTS_UPDATED, false);
        if (success) {
          scanSuccess();
        } else {
          scanFailure();
        }
      }
    };

    // Register the configure WiFi scan receiver
    IntentFilter intentFilter = new IntentFilter();
    intentFilter.addAction(WifiManager.SCAN_RESULTS_AVAILABLE_ACTION);
    getApplicationContext().registerReceiver(wifiScanReceiver, intentFilter);

    // initialize the TaskRunner that is used for asynchronous handling of file writing etc.
    taskRunner = new TaskRunner();

    // start the sensor recording (WiFi + IMU)
    startSensorRecording();

  }

  //
  //  Main entry point for logging all Poses of ARCore camera object and detected augmented images
  //  For every camera frame that we receive we log
  //    - The current camera pose
  //    - All poses of currently tracked augmented images (once an AugImg is detected it is tracked
  //      even if it leaves the camera view
  //

  @Override
  public void onDrawFrame(GL10 gl) {
    // Clear screen to notify driver it should not load any pixels from previous frame.
    GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

    if (session == null) {
      return;
    }

    // Synchronize here to avoid calling Session.update or Session.acquireCameraImage while paused.
    synchronized (frameImageInUseLock) {
      // Notify ARCore session that the view size changed so that the perspective matrix and
      // the video background can be properly adjusted.
      cpuImageDisplayRotationHelper.updateSessionIfNeeded(session);

      try {
        session.setCameraTextureName(cpuImageRenderer.getTextureId());
        final Frame frame = session.update();
        final Camera camera = frame.getCamera();

        // check for augmented images
        checkForDetectedImages(frame);

        long timestamp = SystemClock.elapsedRealtimeNanos();

        // log all poses of the currently tracked augmented images for the current camera frame
        for (AugmentedImage img : trackedImages) {
          Pose aP = img.getCenterPose();
          String s = String.format("%d; %d; %f; %f; %f; %f; %f; %f; %f\n",
                  img.getIndex(), timestamp,
                  aP.tx(), aP.ty(), aP.tz(),
                  aP.qx(), aP.qy(), aP.qz(), aP.qw());
          initPoseLines.add(s);
        }

        if (writeFiles) {

          // obtain the current camera pose and add to the write stack
          Pose pose = frame.getCamera().getPose();

          String s = String.format("%d; %f; %f; %f; %f; %f; %f; %f\n",
                  timestamp,
                  pose.tx(), pose.ty(), pose.tz(),
                  pose.qx(), pose.qy(), pose.qz(), pose.qw());

          poseLines.add(s);

          // write logged poses as batches of size 10 via async tasks to avoid lagging
          if (poseLines.size() > 10) {
            List<String> lines = (ArrayList<String>) ((ArrayList<String>) poseLines).clone();
            poseLines.clear();
            taskRunner.executeAsync(new WriteSensorReadings(lines, poseWriter), (idx) -> { });
          }

          // write logged augmented image poses
          if (initPoseLines.size() > 10) {
            List<String> lines = (ArrayList<String>) ((ArrayList<String>) initPoseLines).clone();
            initPoseLines.clear();
            taskRunner.executeAsync(new WriteSensorReadings(lines, initPoseWriter), (idx) -> { });
          }
        }

        // Keep the screen unlocked while tracking, but allow it to lock when tracking stops.
        trackingStateHelper.updateKeepScreenOnFlag(camera.getTrackingState());

        renderFrameTimeHelper.nextFrame();
        renderProcessedImageCpuDirectAccess(frame);

        // Update the camera intrinsics' text.
        runOnUiThread(() -> cameraIntrinsicsTextView.setText(getCameraIntrinsicsText(frame)));
      } catch (Exception t) {
        // Avoid crashing the application due to unhandled exceptions.
        Log.e(TAG, "Exception on the OpenGL thread", t);
      }
    }
  }

  //
  // Data recording
  //

  private void startSensorRecording() {
    // setup the sensor recording of
    // - WiFi (via android OS)
    // - IMU (via android OS)

    try {
      // Obtain current date for setting filenames
      Calendar c = Calendar.getInstance();
      SimpleDateFormat dateformat = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss");
      String datetime = dateformat.format(c.getTime());

      // Setup file storage location and create folders if not present
      String dirName = getExternalFilesDir(null).getAbsolutePath() + "/" + datetime;
      File dir = new File(dirName);

      if (!dir.exists()) {
        dir.mkdirs();
      }

      // initialize file writers and set instance variables
      writer = new FileWriter(new File(dirName + "/", "sensors.csv"));
      wifiWriter = new FileWriter(new File(dirName + "/", "wifi.csv"));
      poseWriter = new FileWriter(new File(dirName + "/", "poses.csv"));
      initPoseWriter = new FileWriter(new File(dirName + "/", "initPoses.csv"));
      refMarkerWriter = new FileWriter(new File(dirName + "/" + "refMarker.csv"));

    } catch (IOException e) {
      e.printStackTrace();
    }

    // request the first WiFi scan (will be repeatedly request once a scan is received)
    wifiManager.startScan();

    // setup this activity for listing on all required sensor (IMU) events
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_GAME);
    manager.registerListener(VIslam2tagActivity.this, manager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR), SensorManager.SENSOR_DELAY_GAME);

  }

  private void stopRecording() {
    manager.flush(VIslam2tagActivity.this);
    manager.unregisterListener(VIslam2tagActivity.this);
    try {
      writer.close();
      wifiWriter.close();
      poseWriter.close();
      initPoseWriter.close();
      refMarkerWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  //
  // WiFi scan handler methods
  //

  private void scanSuccess() {
    // WiFi scan was successful
    // request the most recent scan and write to file as async task to avoid blocking main thread
    List<ScanResult> results = wifiManager.getScanResults();
    taskRunner.executeAsync(new WriteWLAN(results, wifiWriter), (idx) -> {});

    // update RSS counter for debug purposes
    rssCounter ++;
    rssTextView.setText(String.valueOf(rssCounter));

    // request new WiFi scan
    if (writeFiles) {
      wifiManager.startScan();
    }

  }

  private void scanFailure() {
    // handle failure: new scan did NOT succeed
    // only request new scan but do not use old data here!
    if (writeFiles) {
      wifiManager.startScan();
    }

  }


  //
  // IMU (android sensor) handler
  //

  @Override
  public void onSensorChanged(SensorEvent evt) {

    // store sensor events in local data structure
    if(writeFiles) {
      switch(evt.sensor.getType()) {
        case Sensor.TYPE_ACCELEROMETER:
          sensorLines.add(String.format("%d; ACC; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], 0.f, 0.f, 0.f));
          // writer.write(String.format("%d; ACC; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], 0.f, 0.f, 0.f));
          break;
        case Sensor.TYPE_GYROSCOPE_UNCALIBRATED:
          sensorLines.add(String.format("%d; GYRO_UN; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], evt.values[3], evt.values[4], evt.values[5]));
          break;
        case Sensor.TYPE_GYROSCOPE:
          sensorLines.add(String.format("%d; GYRO; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], 0.f, 0.f, 0.f));
          break;
        case Sensor.TYPE_MAGNETIC_FIELD:
          sensorLines.add(String.format("%d; MAG; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], 0.f, 0.f, 0.f));
          break;
        case Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED:
          sensorLines.add(String.format("%d; MAG_UN; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], 0.f, 0.f, 0.f));
          break;
        case Sensor.TYPE_ROTATION_VECTOR:
          sensorLines.add(String.format("%d; ROT; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], evt.values[3], 0.f, 0.f));
          break;
        case Sensor.TYPE_GAME_ROTATION_VECTOR:
          sensorLines.add(String.format("%d; GAME_ROT; %f; %f; %f; %f; %f; %f\n", evt.timestamp, evt.values[0], evt.values[1], evt.values[2], evt.values[3], 0.f, 0.f));
          break;
      }

      // write locally accumulated sensor reading to disk as batch of size 100
      // use async task to avoid blocking main thread
      if (sensorLines.size() > 100) {
        List<String> lines = (ArrayList<String>) ((ArrayList<String>) sensorLines).clone();
        sensorLines.clear();
        taskRunner.executeAsync(new WriteSensorReadings(lines, writer), (idx) -> { });
      }
    }
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int i) {

  }

  @Override
  public void onFlushCompleted(Sensor sensor) {

  }

  //
  //  Augmented image hanlding (registering + check for updates)
  //

  private AugmentedImageDatabase loadAugDatabase() {
    // load the preconfigured augmented image database
    // since this is much faster than generating it on the fly
    // database has to be created "arcoreimg" tool
    // see: https://developers.google.com/ar/develop/augmented-images/arcoreimg for details
    AugmentedImageDatabase imageDatabase = null;
    try (InputStream inputStream = this.getAssets().open("myimages.imgdb")) {
      imageDatabase = AugmentedImageDatabase.deserialize(session, inputStream);
    } catch (IOException e) {
      // The Augmented Image database could not be deserialized; handle this error appropriately.
    }

    return imageDatabase;

  }

  private void checkForDetectedImages(Frame frame) {
    Collection<AugmentedImage> updatedAugmentedImages =
            frame.getUpdatedTrackables(AugmentedImage.class);

    // check whether Augmented image changed to state tracking and is not currently tracked
    // if this is the case: notify the user that the img was seen for the first time
    for (AugmentedImage img : updatedAugmentedImages) {
      if (img.getTrackingState() == TrackingState.TRACKING) {

        if (!markerIdx.contains(img.getIndex())) {
          markerIdx.add(img.getIndex());

          String notification = "IMG:" + img.getIndex() + " detected";
          runOnUiThread(() -> {
            Toast.makeText(getApplicationContext(), notification, Toast.LENGTH_SHORT).show();
          });
        }

        if (!trackedImages.contains(img)) {
          trackedImages.add(img);
        }

      }
    }
  }

  //
  //  Callback methods for UI interaction
  //

  private void registerClickedMarker() {
    // Log the current timestamp at the time the Reference Marker button was clicked
    // This can be utilized for evaluation of the accuracy by clicking the button once
    // the user is located at a certain known(!) reference location
    // see the paper for details on how it is used
    long timestamp = SystemClock.elapsedRealtimeNanos();
    try {
      refMarkerWriter.write(String.format("%d; %d\n", markerCounter, timestamp));
    } catch (IOException e) {
      e.printStackTrace();
    }
    markerCounter ++;

  }

  //
  // CPU rendering Code from computervision_java examples
  //

  private void setupCPUrendering() {
    cpuImageDisplayRotationHelper = new CpuImageDisplayRotationHelper(/*context=*/ this);

    // Set up renderer.
    surfaceView.setPreserveEGLContextOnPause(true);
    surfaceView.setEGLContextClientVersion(2);
    surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0); // Alpha used for plane blending.
    surfaceView.setRenderer(this);
    surfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    surfaceView.setWillNotDraw(false);

    getLifecycle().addObserver(renderFrameTimeHelper);
    getLifecycle().addObserver(cpuImageFrameTimeHelper);

    installRequested = false;
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus);
  }

  @Override
  public void onPointerCaptureChanged(boolean hasCapture) {

  }

  @Override
  public void onSurfaceCreated(GL10 gl, EGLConfig config) {
    GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Create the texture and pass it to ARCore session to be filled during update().
    try {
      cpuImageRenderer.createOnGlThread(/* context= */ this);
    } catch (IOException e) {
      Log.e(TAG, "Failed to read an asset file", e);
    }
  }

  @Override
  public void onSurfaceChanged(GL10 gl, int width, int height) {
    cpuImageDisplayRotationHelper.onSurfaceChanged(width, height);
    GLES20.glViewport(0, 0, width, height);
  }

  /* Demonstrates how to access a CPU image directly from ARCore. */
  private void renderProcessedImageCpuDirectAccess(Frame frame) {
    try (Image image = frame.acquireCameraImage()) {
      if (image.getFormat() != ImageFormat.YUV_420_888) {
        throw new IllegalArgumentException(
                "Expected image in YUV_420_888 format, got format " + image.getFormat());
      }

      // Do not process the image with edge dectection algorithm if it is not being displayed.
      ByteBuffer processedImageBytesGrayscale =
              edgeDetector.detect(
                      image.getWidth(),
                      image.getHeight(),
                      image.getPlanes()[0].getRowStride(),
                      image.getPlanes()[0].getBuffer());

      cpuImageRenderer.drawWithCpuImage(
              frame,
              image.getWidth(),
              image.getHeight(),
              processedImageBytesGrayscale,
              cpuImageDisplayRotationHelper.getViewportAspectRatio(),
              cpuImageDisplayRotationHelper.getCameraToDisplayRotation());

      // Measure frame time since last successful execution of drawWithCpuImage().
      cpuImageFrameTimeHelper.nextFrame();
    } catch (NotYetAvailableException e) {
      // This exception will routinely happen during startup, and is expected. cpuImageRenderer
      // will handle null image properly, and will just render the background.
      cpuImageRenderer.drawWithoutCpuImage();
    }
  }

  private void setCameraResolution() {
    // First obtain the session handle before getting the list of various camera configs.
    if (session != null) {
      // Create filter here with desired fps filters.
      CameraConfigFilter cameraConfigFilter =
              new CameraConfigFilter(session)
                      .setTargetFps(
                              EnumSet.of(
                                      CameraConfig.TargetFps.TARGET_FPS_30, CameraConfig.TargetFps.TARGET_FPS_60));
      List<CameraConfig> cameraConfigs = session.getSupportedCameraConfigs(cameraConfigFilter);
      Log.i(TAG, "Size of supported CameraConfigs list is " + cameraConfigs.size());

      // Take the first three camera configs, if camera configs size are larger than 3.
      List<CameraConfig> cameraConfigsByResolution =
              new ArrayList<>(
                      cameraConfigs.subList(0, Math.min(cameraConfigs.size(), 3)));
      Collections.sort(
              cameraConfigsByResolution,
              (CameraConfig p1, CameraConfig p2) ->
                      Integer.compare(p1.getImageSize().getHeight(), p2.getImageSize().getHeight()));
      CameraConfig cpuHighResolutionCameraConfig = cameraConfigsByResolution.get(2);
      session.setCameraConfig(cpuHighResolutionCameraConfig);
    }
  }

  private String getCameraIntrinsicsText(Frame frame) {
    Camera camera = frame.getCamera();

    CameraIntrinsics intrinsics = camera.getImageIntrinsics();
    String intrinsicsLabel = "Image";
    String imageType = "CPU";

    float[] focalLength = intrinsics.getFocalLength();
    float[] principalPoint = intrinsics.getPrincipalPoint();
    int[] imageSize = intrinsics.getImageDimensions();

    float fovX = (float) (2 * Math.atan2((double) imageSize[0], (double) (2 * focalLength[0])));
    float fovY = (float) (2 * Math.atan2((double) imageSize[1], (double) (2 * focalLength[1])));
    fovX *= RADIANS_TO_DEGREES;
    fovY *= RADIANS_TO_DEGREES;

    return String.format(
            CAMERA_INTRINSICS_TEXT_FORMAT,
            imageType,
            intrinsicsLabel,
            focalLength[0],
            focalLength[1],
            principalPoint[0],
            principalPoint[1],
            imageType,
            imageSize[0],
            imageSize[1],
            fovX,
            fovY,
            renderFrameTimeHelper.getSmoothedFrameTime(),
            renderFrameTimeHelper.getSmoothedFrameRate(),
            cpuImageFrameTimeHelper.getSmoothedFrameTime(),
            cpuImageFrameTimeHelper.getSmoothedFrameRate(),
            camera.getPose().toString());
  }

  //
  // Activity lifecycle (mostly from computervision_java example)
  // with small adoptions for registering/unregistering listeners
  //

  @Override
  protected void onDestroy() {
    if (session != null) {
      // Explicitly close ARCore Session to release native resources.
      // Review the API reference for important considerations before calling close() in apps with
      // more complicated lifecycle requirements:
      // https://developers.google.com/ar/reference/java/arcore/reference/com/google/ar/core/Session#close()
      session.close();
      session = null;
    }

    stopRecording();

    super.onDestroy();
  }

  @Override
  protected void onResume() {
    super.onResume();

    if (session == null) {
      Exception exception = null;
      String message = null;
      try {
        switch (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
          case INSTALL_REQUESTED:
            installRequested = true;
            return;
          case INSTALLED:
            break;
        }

        // ARCore requires camera permissions to operate. If we did not yet obtain runtime
        // permission on Android M and above, now is a good time to ask the user for it.
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
          CameraPermissionHelper.requestCameraPermission(this);
          return;
        }

        session = new Session(/* context= */ this);
        config = new Config(session);
      } catch (UnavailableArcoreNotInstalledException
              | UnavailableUserDeclinedInstallationException e) {
        message = "Please install ARCore";
        exception = e;
      } catch (UnavailableApkTooOldException e) {
        message = "Please update ARCore";
        exception = e;
      } catch (UnavailableSdkTooOldException e) {
        message = "Please update this app";
        exception = e;
      } catch (Exception e) {
        message = "This device does not support AR";
        exception = e;
      }

      if (message != null) {
        messageSnackbarHelper.showError(this, message);
        Log.e(TAG, "Exception creating session", exception);
        return;
      }
    }

    config.setFocusMode(Config.FocusMode.AUTO);

    AugmentedImageDatabase db = loadAugDatabase();
    if (db != null) config.setAugmentedImageDatabase(db);

    session.configure(config);
    setCameraResolution();

    // Note that order matters - see the note in onPause(), the reverse applies here.
    try {
      session.resume();
    } catch (CameraNotAvailableException e) {
      messageSnackbarHelper.showError(this, "Camera not available. Try restarting the app.");
      session = null;
      return;
    }
    surfaceView.onResume();
    cpuImageDisplayRotationHelper.onResume();

  }

  @Override
  public void onPause() {
    super.onPause();

    if (session != null) {
      // Note that the order matters - GLSurfaceView is paused first so that it does not try
      // to query the session. If Session is paused before GLSurfaceView, GLSurfaceView may
      // still call session.update() and get a SessionPausedException.
      cpuImageDisplayRotationHelper.onPause();
      surfaceView.onPause();
      session.pause();
    }
    //
  }

  //
  // Permission handling (camera + location for WiFi scans)
  //

  private void requestAppPermissions() {
    if (ContextCompat.checkSelfPermission(
            getApplicationContext(), Manifest.permission.ACCESS_FINE_LOCATION) ==
            PackageManager.PERMISSION_GRANTED && ContextCompat.checkSelfPermission(
            getApplicationContext(), Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED) {
    } else {
      // You can directly ask for the permission.
      // The registered ActivityResultCallback gets the result of this request.
      ActivityCompat.requestPermissions(VIslam2tagActivity.this,
              new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.CAMERA},
              1);
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
    super.onRequestPermissionsResult(requestCode, permissions, results);
    if (!CameraPermissionHelper.hasCameraPermission(this)) {
      Toast.makeText(this, "Camera permission is needed to run this application", Toast.LENGTH_LONG)
              .show();
      if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
        // Permission denied with checking "Do not ask again".
        CameraPermissionHelper.launchPermissionSettings(this);
      }
      finish();
    }
  }



}
