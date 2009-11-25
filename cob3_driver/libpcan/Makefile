all: libpcan

TARBALL = build/peak-linux-driver-6.11.tar.gz
TARBALL_URL = http://p103112.typo3server.info/fileadmin/media/linux/files/peak-linux-driver.6.11.tar.gz
SOURCE_DIR = build/peak-linux-driver-6.11
include $(shell rospack find mk)/download_unpack_build.mk

libpcan: $(SOURCE_DIR)/unpacked
	# build
	cd $(SOURCE_DIR) && make
	# build driver
	sudo ln -f -s /usr/src/linux-headers*-generic /usr/src/linux
	cd $(SOURCE_DIR)/driver && make
	# copy to src
	mkdir -p src
	cp $(SOURCE_DIR)/lib/lib* src
clean:
	rm -rf src $(SOURCE_DIR)
wipe: clean
	-rm -rf build
