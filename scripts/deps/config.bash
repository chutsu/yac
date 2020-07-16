# Configurations
PREFIX="/usr/local"
DOWNLOAD_PATH="$PREFIX/src"
BUILD_TYPE="Release"

apt_update() {
  echo "[Updating APT package list]";
  apt-get update -qqq;
}

apt_install() {
  apt-get install -qqq -y "$@";
}

# $1 - Git Repo URL
# $2 - Repo folder name
install_git_repo() {
    # Clone repo
    mkdir -p $DOWNLOAD_PATH
    cd $DOWNLOAD_PATH || return
    if [ ! -d "$2" ]; then
      git clone "$1" "$2"
    fi

    # Go into repo
    cd "$2" || return
    # git pull

    # Prepare for build
    mkdir -p build
    cd build || return
    cmake .. \
      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      -DCMAKE_INSTALL_PREFIX=$PREFIX \
      $CMAKE_EXTRA_ARGS

    # Compile and install
    make -j2 && make install
}

# $1 - Mercurial Repo URL
# $2 - Repo folder name
install_hg_repo() {
    # Clone repo
    mkdir -p $DOWNLOAD_PATH
    cd "$DOWNLOAD_PATH" || return
    if [ ! -d "$2" ]; then
      hg clone "$1" "$2"
    fi

    # Go into repo
    cd "$2" || return
    hg pull

    # Prepare for build
    mkdir -p build
    cd build || return
    cmake .. \
      -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
      -DCMAKE_INSTALL_PREFIX=$PREFIX \
      $CMAKE_EXTRA_ARGS

    # Compile and install
    make -j2 && make install
}

# "$1" - Repo URL
# "$2" - Repo folder name
install_zip_repo() {
  # Download repo
  mkdir -p $DOWNLOAD_PATH
  cd $DOWNLOAD_PATH || return
  if [ ! -f "$2".zip ]; then
    wget --no-check-certificate "$1" -O "$2".zip
  fi
  unzip -oqq "$2".zip
  cd "$2" || return

  # Compile and install opencv
  mkdir -p build
  cd build || return
  cmake .. \
    -DCMAKE_BUILD_TYPE=$BUILD_TYPE \
    -DCMAKE_INSTALL_PREFIX=$PREFIX \
    $CMAKE_EXTRA_ARGS

  make -j"$(nproc)" && make install
}
