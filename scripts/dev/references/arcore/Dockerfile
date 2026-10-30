FROM eclipse-temurin@sha256:e0c60c487345d1dc9d0fc7b6f0496f3cc941e5132e09296cc17a6decc71b902b
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3 python3-venv unzip curl socat ca-certificates libgl1 libglu1-mesa libnss3 \
    libx11-6 libxcb1 libxcomposite1 libxcursor1 libxi6 libxrandr2 libxrender1 libxtst6 \
    libxkbcommon0 libpulse0 libasound2 libdbus-1-3 libfontconfig1 libfreetype6 \
    libxshmfence1 libdrm2 libgbm1 libvulkan1 mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*
ENV ANDROID_HOME=/work/toolchain/sdk \
    ANDROID_SDK_ROOT=/work/toolchain/sdk \
    ANDROID_AVD_HOME=/work/toolchain/avd \
    ANDROID_USER_HOME=/work/toolchain/android-user \
    GRADLE_USER_HOME=/work/toolchain/gradle
ENV PATH=/work/toolchain/sdk/cmdline-tools/latest/bin:/work/toolchain/sdk/platform-tools:/work/toolchain/sdk/emulator:${PATH}
WORKDIR /work
CMD ["sleep", "infinity"]
