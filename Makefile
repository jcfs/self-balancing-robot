INO=ino

all: build upload serial

build:
	$(INO) build

upload:
	sudo $(INO) upload

serial: 
	sudo $(INO) serial -b 115200

clean:
	$(INO) clean
