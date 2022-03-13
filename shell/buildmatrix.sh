echo "Initializing Buildmatrix"
matrix_priv_file=""
function matrix/priv/build() {
    echo "Build debug $1"
    ./gradlew assembleDebug
    mv TeamCode/build/outputs/apk/debug/TeamCode-debug.apk TeamCode-debug-"$1".apk
    echo "Build release $1"
    ./gradlew assembleRelease
    mv TeamCode/build/outputs/apk/release/TeamCode-release.apk TeamCode-release-"$1".apk
}

function matrix/priv/config() {
  echo "Using configuration $1"
  local tmp_config=$(mktemp)
  sed "s/RobotConfig CURRENT = .*;/RobotConfig CURRENT = $1;/" "$matrix_priv_file" > "$tmp_config"
  cat "$tmp_config" > "$matrix_priv_file"
  rm "$tmp_config"
}

function matrix/build() {
  matrix/priv/config "$1"
  matrix/priv/build "$1"
}

function matrix/config() {
  echo "Storing configuration in $1"
  matrix_priv_file="$1"
}