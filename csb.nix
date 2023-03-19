with import <nixpkgs> {};

let
    android-nixpkgs = callPackage (import (builtins.fetchGit {
        url = "https://github.com/tadfisher/android-nixpkgs.git";
    })) {
        # Default; can also choose "beta", "preview", or "canary".
        channel = "stable";
    };
    
    android-sdk = android-nixpkgs.sdk (sdkPkgs: with sdkPkgs; [
        cmdline-tools-latest
        build-tools-30-0-2
        platform-tools
        platforms-android-29
        emulator
    ]);
    
in stdenv.mkDerivation {
    name = "csb";
    buildInputs = [
        zulu
        kotlin
        kotlin-language-server
        android-tools
        android-sdk
    ];
}