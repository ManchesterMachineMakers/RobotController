with import <nixpkgs> {};

stdenv.mkDerivation {
    name = "csb";
    buildInputs = [
        zulu
        kotlin
        adoptopenjdk-hotspot-bin
    ];
}