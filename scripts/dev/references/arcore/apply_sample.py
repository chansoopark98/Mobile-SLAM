#!/usr/bin/env python3
"""Add an observer to the pinned Apache-2.0 HelloAR sample; never alter ARCore."""
import hashlib,json,pathlib,shutil,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[4]
SAMPLE=ROOT/'assets/references/ARCore/source/samples/hello_ar_java'
JAVA=SAMPLE/'app/src/main/java/com/google/ar/core/examples/java/helloar'
path=JAVA/'HelloArActivity.java';source=path.read_text()
if 'ReferenceAudit referenceAudit' in source:raise RuntimeError('Observer already applied; inspect existing patch')
source=source.replace('  private final TrackingStateHelper trackingStateHelper', '  private final ReferenceAudit referenceAudit = new ReferenceAudit(this);\n  private final TrackingStateHelper trackingStateHelper')
source=source.replace('      session.resume();','      referenceAudit.prepare(session);\n      session.resume();')
source=source.replace('      frame = session.update();','      long referenceUpdateStarted = android.os.SystemClock.elapsedRealtimeNanos();\n      frame = session.update();\n      referenceAudit.observe(session, frame, android.os.SystemClock.elapsedRealtimeNanos() - referenceUpdateStarted);')
source=source.replace('CUBEMAP_NUMBER_OF_IMPORTANCE_SAMPLES = 32','CUBEMAP_NUMBER_OF_IMPORTANCE_SAMPLES = 16')
path.write_text(source);shutil.copy2(pathlib.Path(__file__).with_name('ReferenceAudit.java'),JAVA/'ReferenceAudit.java')
# Public SDK package repository does not provide android-37 in this environment.
# API35 + AGP8.6.1/Gradle8.7 is an integration build profile, not an engine change.
app=SAMPLE/'app/build.gradle';app.write_text(app.read_text().replace('compileSdkVersion 37','compileSdkVersion 35').replace('targetSdkVersion 37','targetSdkVersion 35'))
root=SAMPLE/'build.gradle';root.write_text(root.read_text().replace('com.android.tools.build:gradle:8.4.0','com.android.tools.build:gradle:8.6.1'))
wrapper=SAMPLE/'gradle/wrapper/gradle-wrapper.properties';wrapper.write_text(wrapper.read_text().replace('gradle-8.6-bin.zip','gradle-8.7-bin.zip'))
manifest=SAMPLE/'app/src/main/AndroidManifest.xml';text=manifest.read_text();text=text.replace('<uses-permission android:name="android.permission.CAMERA" />','<uses-permission android:name="android.permission.CAMERA" />\n    <uses-permission android:name="android.permission.HIGH_SAMPLING_RATE_SENSORS" />');manifest.write_text(text)
EVIDENCE=ROOT/'build/research-20261012/commercial-experiments';patch=subprocess.check_output(['git','diff','--binary'],cwd=ROOT/'assets/references/ARCore/source');(EVIDENCE/'official-sample-observer.patch').write_bytes(patch)
files=[path,JAVA/'ReferenceAudit.java',app,root,wrapper,manifest];(EVIDENCE/'official-sample-source-manifest.json').write_text(json.dumps({'upstreamCommit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT/'assets/references/ARCore/source',text=True).strip(),'observerScope':'public SDK output/CPU image/Android sensor observer only; ARCore binary unchanged','compatibilityProfile':{'compileSdk':35,'targetSdk':35,'agp':'8.6.1','gradle':'8.7','arcore':'1.56.0'},'files':[{'path':str(f),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()}for f in files]},indent=2)+'\n');print('Applied public SDK observer and explicit sample build compatibility profile')
