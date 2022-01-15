echo "Initializing Buildmatrix"
matrix_priv_file=""
function matrix/priv/build() {
    echo "Build debug $1"
    ./gradlew assembleDebug
    mv TeamCode/build/outputs/apk/debug/TeamCode-debug.apk TeamCode-debug-$1.apk
    echo "Build release $1"
    ./gradlew assembleRelease
    mv TeamCode/build/outputs/apk/release/TeamCode-release.apk TeamCode-release-$1.apk
}

function matrix/priv/config() {
  echo "Using configuration $1"
  echo $matrix_priv_file
  sed -i .prev_config "s/RobotHardware CURRENT = .*;/RobotHardware CURRENT = $1;/" "$matrix_priv_file"
}

function matrix/build() {
  matrix/priv/config "$1"
  matrix/priv/build "$1"
}

function matrix/config() {
  echo "Storing configuration in $1"
  matrix_priv_file="$1"
}