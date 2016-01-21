INO=ino

all:
	$(INO) build && sudo $(INO) upload 

serial: 
	sudo $(INO) serial -b 115200

clean:
	$(INO) clean
