$ErrorActionPreference = "Stop"
if (-not $env:ANDROID_NDK_HOME) {
    $env:ANDROID_NDK_HOME = "C:\Android\ndk-27.2"
}
if (-not (Test-Path "$env:ANDROID_NDK_HOME\ndk-build.cmd")) {
    throw "ndk-build.cmd not found at $env:ANDROID_NDK_HOME. Set ANDROID_NDK_HOME to your NDK folder."
}
& "$env:ANDROID_NDK_HOME\ndk-build.cmd" `
  NDK_PROJECT_PATH="$PWD" `
  APP_BUILD_SCRIPT="$PWD\Android.mk" `
  NDK_APPLICATION_MK="$PWD\Application.mk" `
  -j4
