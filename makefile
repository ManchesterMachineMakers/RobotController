.DEFAULT_TARGET := build
build: TeamCode
	./gradlew assembleDebug
install: TeamCode
	./gradlew installDebug
