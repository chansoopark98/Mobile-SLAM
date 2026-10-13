package com.google.ar.core.examples.java.helloar;

import android.app.Activity;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.Image;
import android.net.Uri;
import android.os.SystemClock;
import android.util.Log;
import com.google.ar.core.Camera;
import com.google.ar.core.CameraIntrinsics;
import com.google.ar.core.Frame;
import com.google.ar.core.PlaybackStatus;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;
import com.google.ar.core.RecordingConfig;
import com.google.ar.core.RecordingStatus;
import com.google.ar.core.Session;
import com.google.ar.core.TrackingState;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.nio.ByteBuffer;
import java.security.MessageDigest;
import org.json.JSONArray;
import org.json.JSONObject;

/** Public-SDK experiment observer; does not inject or modify ARCore state. */
final class ReferenceAudit {
  private static final String TAG = "ARCORE_COMPARE";
  private final Activity activity;
  private final Object lock = new Object();
  private boolean prepared;
  private boolean finished;
  private String mode;
  private PrintWriter poses;
  private PrintWriter imu;
  private SensorManager sensors;
  private SensorEventListener listener;
  private int frames;
  private int tracking;
  private int images;
  private int imuEvents;
  private long previousTimestamp = -1;
  private String setupError;
  private File recording;

  ReferenceAudit(Activity activity) { this.activity = activity; }

  void prepare(Session session) {
    if (prepared) return;
    mode = activity.getIntent().getStringExtra("compare_mode");
    if (mode == null) return;
    prepared = true;
    try {
      recording = new File(activity.getFilesDir(), "reference-recording.mp4");
      poses = new PrintWriter(new File(activity.getFilesDir(), "reference-" + mode + "-poses.jsonl"));
      imu = new PrintWriter(new File(activity.getFilesDir(), "reference-" + mode + "-imu.jsonl"));
      if (mode.equals("record")) {
        session.startRecording(new RecordingConfig(session)
            .setMp4DatasetUri(Uri.fromFile(recording)).setAutoStopOnPause(true));
      } else if (mode.startsWith("playback")) {
        session.setPlaybackDatasetUri(Uri.fromFile(recording));
      }
      JSONObject config = new JSONObject();
      config.put("event", "setup");
      config.put("mode", mode);
      config.put("updateMode", session.getConfig().getUpdateMode().name());
      config.put("recordingStatus", session.getRecordingStatus().name());
      config.put("playbackStatus", session.getPlaybackStatus().name());
      poses.println(config.toString());
      if (mode.startsWith("playback")) {
        Log.i(TAG, "playback IMU stays inside ARCore MP4; live SensorManager is not recorded playback data");
        return;
      }
      sensors = (SensorManager) activity.getSystemService(Activity.SENSOR_SERVICE);
      listener = new SensorEventListener() {
        @Override public void onAccuracyChanged(Sensor sensor, int accuracy) {}
        @Override public void onSensorChanged(SensorEvent event) {
          synchronized (lock) {
            if (finished || imu == null) return;
            try {
              JSONObject row = new JSONObject();
              row.put("source", "AndroidSensorManager_live_virtual_device_observer");
              row.put("sensorType", event.sensor.getType());
              row.put("timestampNs", Long.toString(event.timestamp));
              row.put("arrivalElapsedNs", Long.toString(SystemClock.elapsedRealtimeNanos()));
              row.put("accuracy", event.accuracy);
              JSONArray values = new JSONArray();
              for (float value : event.values) values.put(value);
              row.put("values", values);
              imu.println(row.toString());
              ++imuEvents;
            } catch (Exception error) { Log.e(TAG, "IMU observer", error); }
          }
        }
      };
      for (int type : new int[] {Sensor.TYPE_ACCELEROMETER, Sensor.TYPE_GYROSCOPE}) {
        Sensor sensor = sensors.getDefaultSensor(type);
        Log.i(TAG, "sensor " + type + " available=" + (sensor != null));
        if (sensor != null) sensors.registerListener(listener, sensor, SensorManager.SENSOR_DELAY_FASTEST);
      }
      Log.i(TAG, "prepared " + mode);
    } catch (Exception error) {
      setupError = error.toString();
      Log.e(TAG, "setup failed", error);
    }
  }

  void observe(Session session, Frame frame, long updateDurationNs) {
    if (!prepared || finished) return;
    try {
      long timestamp = frame.getTimestamp();
      if (mode.startsWith("playback") && session.getPlaybackStatus() == PlaybackStatus.FINISHED) {
        finish(session, "playback_finished");
        return;
      }
      if (timestamp <= 0 || timestamp == previousTimestamp) return;
      previousTimestamp = timestamp;
      Camera camera = frame.getCamera();
      Pose pose = camera.getPose();
      CameraIntrinsics intrinsics = camera.getImageIntrinsics();
      JSONObject row = new JSONObject();
      row.put("event", "frame");
      row.put("index", frames++);
      row.put("timestampNs", Long.toString(timestamp));
      row.put("arrivalElapsedNs", Long.toString(SystemClock.elapsedRealtimeNanos()));
      row.put("sessionUpdateDurationNs", Long.toString(updateDurationNs));
      row.put("trackingState", camera.getTrackingState().name());
      row.put("trackingFailureReason", camera.getTrackingFailureReason().name());
      row.put("recordingStatus", session.getRecordingStatus().name());
      row.put("playbackStatus", session.getPlaybackStatus().name());
      if (camera.getTrackingState() == TrackingState.TRACKING) ++tracking;
      JSONArray translation = new JSONArray();
      for (float value : pose.getTranslation()) translation.put(value);
      JSONArray quaternion = new JSONArray();
      for (float value : pose.getRotationQuaternion()) quaternion.put(value);
      float[] matrix = new float[16];
      pose.toMatrix(matrix, 0);
      JSONArray transform = new JSONArray();
      for (float value : matrix) transform.put(value);
      row.put("translationM", translation);
      row.put("quaternionXyzw", quaternion);
      row.put("matrixColumnMajorT_W_C_OpenGL", transform);
      row.put("focalLengthPx", new JSONArray(intrinsics.getFocalLength()));
      row.put("principalPointPx", new JSONArray(intrinsics.getPrincipalPoint()));
      row.put("cpuImageDimensions", new JSONArray(intrinsics.getImageDimensions()));
      try (PointCloud points = frame.acquirePointCloud()) {
        row.put("pointCloudPoints", points.getPoints().remaining() / 4);
        row.put("pointCloudTimestampNs", Long.toString(points.getTimestamp()));
      }
      if (activity.getIntent().getBooleanExtra("compare_save_images", false)) {
        try (Image image = frame.acquireCameraImage()) {
          int width = image.getWidth();
          int height = image.getHeight();
          Image.Plane plane = image.getPlanes()[0];
          ByteBuffer bytes = plane.getBuffer();
          byte[] gray = new byte[width * height];
          int base = bytes.position();
          for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
              gray[y * width + x] = bytes.get(base + y * plane.getRowStride() + x * plane.getPixelStride());
          File directory = new File(activity.getFilesDir(), "reference-" + mode + "-images");
          directory.mkdirs();
          try (FileOutputStream stream = new FileOutputStream(new File(directory, String.format("%06d.pgm", frames - 1)))) {
            stream.write(("P5\n" + width + " " + height + "\n255\n").getBytes("US-ASCII"));
            stream.write(gray);
          }
          StringBuilder hash = new StringBuilder();
          for (byte value : MessageDigest.getInstance("SHA-256").digest(gray)) hash.append(String.format("%02x", value & 255));
          row.put("imagePayloadSha256", hash.toString());
          row.put("imageTimestampNs", Long.toString(image.getTimestamp()));
          row.put("imageWidth", width);
          row.put("imageHeight", height);
          row.put("imageRowStride", plane.getRowStride());
          row.put("imagePixelStride", plane.getPixelStride());
          ++images;
        } catch (Exception error) { row.put("imageUnavailable", error.getClass().getSimpleName()); }
      }
      synchronized (lock) { if (poses != null) poses.println(row.toString()); }
      if (frames % 30 == 0) Log.i(TAG, "progress " + mode + " frames=" + frames + " tracking=" + tracking);
      if (frames >= activity.getIntent().getIntExtra("compare_frames", 300)) finish(session, "frame_limit");
    } catch (Exception error) { Log.e(TAG, "frame observer failed", error); }
  }

  void finish(Session session, String reason) {
    synchronized (lock) {
      if (!prepared || finished) return;
      finished = true;
      try {
        if (mode.equals("record") && session.getRecordingStatus() == RecordingStatus.OK) session.stopRecording();
        JSONObject summary = new JSONObject();
        summary.put("event", "summary");
        summary.put("mode", mode);
        summary.put("reason", reason);
        summary.put("frames", frames);
        summary.put("trackingFrames", tracking);
        summary.put("savedImages", images);
        summary.put("imuEvents", imuEvents);
        summary.put("externalImuSource", mode.startsWith("playback") ? "unavailable_recorded_imu_internal_to_ARCore" : "AndroidSensorManager_live_observer_not_proven_internal_ARCore_packets");
        summary.put("setupError", setupError == null ? JSONObject.NULL : setupError);
        summary.put("recordingBytes", recording != null && recording.exists() ? recording.length() : 0);
        summary.put("recordingStatus", session.getRecordingStatus().name());
        summary.put("playbackStatus", session.getPlaybackStatus().name());
        if (poses != null) { poses.println(summary.toString()); poses.close(); poses = null; }
        if (imu != null) { imu.close(); imu = null; }
        if (sensors != null && listener != null) sensors.unregisterListener(listener);
        try (PrintWriter out = new PrintWriter(new File(activity.getFilesDir(), "reference-" + mode + "-summary.json"))) { out.println(summary.toString()); }
        Log.i(TAG, "finished " + summary.toString());
      } catch (Exception error) { Log.e(TAG, "finish failed", error); }
      activity.runOnUiThread(() -> activity.finish());
    }
  }
}
