.DEFAULT_TARGET := build
build: TeamCode
	./gradlew assembleDebug
install: TeamCode
	./gradlew installDebug
reboot: install
	adb reboot
restart: install
	adb shell monkey -p com.qualcomm.ftcrobotcontroller 1