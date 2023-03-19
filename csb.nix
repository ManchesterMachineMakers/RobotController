with import <nixpkgs> {};

stdenv.mkDerivation {
    name = "csb";
    buildInputs = [
        openjdk18-bootstrap
        kotlin
        kotlin-language-server
    ];
}