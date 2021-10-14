.DEFAULT_TARGET := all
all:
	@./builder.sh build
install:
	@./builder.sh install
