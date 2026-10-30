#include <VuforiaEngine/VuforiaEngine.h>
#include <jni.h>
#include <string>

static_assert(VU_VERSION_MAJOR == 11 && VU_VERSION_MINOR == 4 && VU_VERSION_PATCH == 4,
              "This reference is pinned to Vuforia 11.4.4");

extern "C" JNIEXPORT jstring JNICALL
Java_com_mobileslam_vuforiareference_MainActivity_sdkVersion(JNIEnv* env, jobject) {
    const auto version = vuEngineGetLibraryVersionInfo();
    const std::string text = std::string("Vuforia SDK library: ") +
        (version.versionString ? version.versionString : "unavailable") +
        "\nHeaders: 11.4.4\nEngine tracking: not activated\nNo camera/IMU data processed.";
    return env->NewStringUTF(text.c_str());
}
