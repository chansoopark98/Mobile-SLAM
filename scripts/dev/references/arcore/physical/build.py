#!/usr/bin/env python3
"""Copy and build the public HelloAR sample without contacting any device."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[5]
HERE = Path(__file__).resolve().parent
PACKAGE = 'com.google.ar.core.examples.java.helloar'
IMAGE = 'sha256:f916020d2978b27752fdf583cefb724a488bd98fb799f7615b0cb7728862388f'


def replace(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f'Expected exactly one patch anchor: {old[:100]!r}')
    return text.replace(old, new)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--keystore', type=Path)
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    original = ROOT / 'assets/references/ARCore/source/samples/hello_ar_java'
    sample = output / 'source'
    shutil.copytree(original, sample, ignore=shutil.ignore_patterns('build', '.gradle', 'local.properties'))
    java = sample / 'app/src/main/java/com/google/ar/core/examples/java/helloar'
    for name in ['PreflightActivity.java', 'MeasurementInstrumentation.java', 'ReferenceAudit.java']:
        shutil.copy2(HERE / name, java / name)
    token = secrets.token_hex(24)
    (output / 'start-token.txt').write_text(token + '\n')
    (java / 'BuildGuard.java').write_text(f'package {PACKAGE};\nfinal class BuildGuard {{ static final String TOKEN = "{token}"; }}\n')
    path = java / 'HelloArActivity.java'
    text = path.read_text()
    text = replace(text, '    super.onCreate(savedInstanceState);', '    super.onCreate(savedInstanceState);\n    if (!referenceAudit.begin()) { finish(); return; }')
    text = replace(text, '    super.onResume();', '    super.onResume();\n    if (referenceAudit.isFinished() || surfaceView == null) return;')
    start = text.index('          switch (ArCoreApk.getInstance().requestInstall')
    end = text.index('\n        }', start)
    text = text[:start] + '          throw new IllegalStateException("Installed ARCore service required; no install/update requested");' + text[end:]
    text = text.replace('CameraPermissionHelper.requestCameraPermission(this);\n          return;', 'throw new IllegalStateException("CAMERA must be granted before measurement entry");')
    text = text.replace('      } catch (UnavailableArcoreNotInstalledException\n          | UnavailableUserDeclinedInstallationException e) {', '      } catch (UnavailableArcoreNotInstalledException e) {')
    text = replace(text, '        Log.e(TAG, "Exception creating session", exception);\n        return;', '        Log.e(TAG, "Exception creating session", exception);\n        stopMeasurement("session_setup_error:" + exception);\n        return;')
    text = replace(text, '    } catch (CameraNotAvailableException e) {\n      messageSnackbarHelper.showError(this, "Camera not available. Try restarting the app.");\n      session = null;\n      return;', '    } catch (Exception e) {\n      stopMeasurement("session_resume_error:" + e);\n      return;')
    start = text.index('    if (session != null) {', text.index('  public void onPause()'))
    end = text.index('\n  }', start)
    text = text[:start] + '    stopMeasurement("activity_pause");' + text[end:]
    start = text.index('    if (session != null) {', text.index('  protected void onDestroy()'))
    end = text.index('\n    super.onDestroy();', start)
    text = text[:start] + '    stopMeasurement("activity_destroy");\n' + text[end:]
    text = replace(text, '    if (session == null) {\n      return;\n    }', '    if (session == null || referenceAudit.isFinished()) {\n      return;\n    }')
    text = replace(text, '      Log.e(TAG, "Camera not available during onDrawFrame", e);', '      referenceAudit.requestStop("frame_camera_error:" + e);\n      Log.e(TAG, "Camera not available during onDrawFrame", e);')
    # Catch renderer runtime failures as well as observer failures; every path requests cleanup.
    text = replace(text, '  public void onDrawFrame(SampleRender render) {', '''  public void onDrawFrame(SampleRender render) {
    try { drawMeasurementFrame(render); }
    catch (Exception error) { referenceAudit.requestStop("render_error:" + error); }
  }

  private void drawMeasurementFrame(SampleRender render) {''')
    anchor = '  /** Menu button to launch feature specific settings. */'
    cleanup = '''  private boolean measurementStopped;

  void stopMeasurement(String reason) {
    if (measurementStopped) return;
    measurementStopped = true;
    String error = null;
    try {
      if (displayRotationHelper != null) displayRotationHelper.onPause();
      if (surfaceView != null) surfaceView.onPause();
    } catch (Exception failure) { error = failure.toString(); }
    referenceAudit.finish(session, reason);
    boolean closed = session == null;
    if (session != null) {
      try { session.pause(); } catch (Exception failure) { error = failure.toString(); }
      try { session.close(); closed = true; } catch (Exception failure) { error = failure.toString(); }
      session = null;
    }
    referenceAudit.lifecycleResult(closed, error);
    if (MeasurementInstrumentation.completed != null) MeasurementInstrumentation.completed.countDown();
    finish();
  }

'''
    text = replace(text, anchor, cleanup + anchor)
    path.write_text(text)
    gradle = sample / 'app/build.gradle'
    text = gradle.read_text().replace('applicationId "com.google.ar.core.examples.java.helloar"', 'applicationId "com.mobileslam.reference.arcore"\n        ndk { abiFilters "arm64-v8a" }')
    text = replace(text, '    buildTypes {', '''    signingConfigs {
        debug {
            storeFile file('/output/debug.keystore')
            storePassword 'android'
            keyAlias 'androiddebugkey'
            keyPassword 'android'
        }
    }
    buildTypes {''')
    gradle.write_text(text)
    manifest = sample / 'app/src/main/AndroidManifest.xml'
    text = manifest.read_text().replace('android.permission.POST_NOTIFICATIONS', 'android.permission.HIGH_SAMPLING_RATE_SENSORS')
    text = text.replace('android:exported="true"', 'android:exported="false"')
    start = text.index('      <intent-filter>')
    end = text.index('      </intent-filter>', start) + len('      </intent-filter>')
    text = text[:start] + text[end:]
    text = text.replace('    <!-- Indicates whether', f'''    <activity android:name="{PACKAGE}.PreflightActivity" android:exported="true">
      <intent-filter>
        <action android:name="android.intent.action.MAIN" />
        <category android:name="android.intent.category.LAUNCHER" />
      </intent-filter>
    </activity>
    <!-- Indicates whether''')
    text = text.replace('</manifest>', f'  <instrumentation android:name="{PACKAGE}.MeasurementInstrumentation" android:targetPackage="com.mobileslam.reference.arcore" />\n</manifest>')
    manifest.write_text(text)
    (sample / 'local.properties').write_text('sdk.dir=/sdk\n')
    if args.keystore:
        shutil.copy2(args.keystore, output / 'debug.keystore')
    snapshots = {str(p.relative_to(sample)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sample.rglob('*') if p.is_file()}
    (output / 'source-manifest.json').write_text(json.dumps(snapshots, indent=2) + '\n')
    shutil.make_archive(str(output / 'source-snapshot'), 'gztar', sample)
    script = '''set -eu
cp -a /cache-template/. /output/gradle-cache/
if [ ! -f /output/debug.keystore ]; then
  keytool -genkeypair -keystore /output/debug.keystore -storepass android -keypass android -alias androiddebugkey -dname 'CN=Mobile SLAM Reference, O=Local Research, C=KR' -keyalg RSA -keysize 2048 -validity 10000
fi
cd /output/source
bash gradlew --offline --no-daemon --max-workers=2 assembleDebug
cp app/build/outputs/apk/debug/app-debug.apk /output/reference-physical.apk
/sdk/build-tools/35.0.0/apksigner verify --print-certs /output/reference-physical.apk > /output/apk-signature.txt
/sdk/build-tools/35.0.0/aapt dump badging /output/reference-physical.apk > /output/apk-badging.txt
/sdk/build-tools/35.0.0/aapt dump xmltree /output/reference-physical.apk AndroidManifest.xml > /output/apk-manifest.txt
sha256sum /output/reference-physical.apk > /output/apk-sha256.txt
'''
    (output / 'container-build.sh').write_text(script)
    argv = ['docker', 'run', '--name', 'mobile-slam-physical-' + output.name, '--label', 'mobile-slam-reference=physical-arcore-20261014', '--network=none', '--cpus=2', '--memory=4g', '-v', f'{output}:/output', '-v', f'{ROOT}/assets/references/ARCore/toolchain/sdk:/sdk:ro', '-v', f'{ROOT}/assets/references/ARCore/toolchain/gradle:/cache-template:ro', '-e', 'ANDROID_HOME=/sdk', '-e', 'ANDROID_SDK_ROOT=/sdk', '-e', 'ANDROID_USER_HOME=/output/android-user', '-e', 'GRADLE_USER_HOME=/output/gradle-cache', IMAGE, 'bash', '/output/container-build.sh']
    (output / 'build-command.json').write_text(json.dumps(argv, indent=2) + '\n')
    with (output / 'build.log').open('x') as log:
        status = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT).returncode
    (output / 'build-result.json').write_text(json.dumps({'exit': status, 'image': IMAGE}) + '\n')
    subprocess.run(['docker', 'run', '--rm', '-v', f'{output}:/output', IMAGE, 'chown', '-R', f'{os.getuid()}:{os.getgid()}', '/output'], check=True)
    print(json.dumps({'output': str(output), 'exit': status}))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
