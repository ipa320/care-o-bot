all: libpcan

TARBALL = build/peak-linux-driver-6.11.tar.gz
TARBALL_URL = http://p103112.typo3server.info/fileadmin/media/linux/files/peak-linux-driver.6.11.tar.gz
SOURCE_DIR = build/peak-linux-driver-6.11
include $(shell rospack find mk)/download_unpack_build.mk

libpcan: $(SOURCE_DIR)/unpacked
	# build
	cd $(SOURCE_DIR) && make
	# build driver, set link to actual headers 
	sudo ln -f -s /usr/src/linux-headers-2.6.31-15-generic /usr/src/linux
	cd $(SOURCE_DIR)/driver && make
	# copy to common
	mkdir -p common
	mkdir -p common/lib
	mkdir -p common/include
	cp $(SOURCE_DIR)/lib/*.so* common/lib
	cp common/lib/libpcan.so common/lib/libpcan.so.0
	cp $(SOURCE_DIR)/lib/*.h common/include
clean:
	rm -rf common $(SOURCE_DIR)
wipe: clean
	-rm -rf build
