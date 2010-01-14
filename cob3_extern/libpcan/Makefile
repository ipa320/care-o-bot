all: libpcan

TARBALL = build/peak-linux-driver-6.11.tar.gz
TARBALL_URL = http://p103112.typo3server.info/fileadmin/media/linux/files/peak-linux-driver.6.11.tar.gz
SOURCE_DIR = build/peak-linux-driver-6.11
KERNEL_VER=/usr/src/linux-headers-"`uname -r`"
include $(shell rospack find mk)/download_unpack_build.mk

libpcan: $(SOURCE_DIR)/unpacked
	# build
	cd $(SOURCE_DIR) && make KERNEL_LOCATION=$(KERNEL_VER)
	# build driver, set link to actual headers 
	#sudo ln -f -s /usr/src/linux-headers-"`uname -r`" /usr/src/linux
	cd $(SOURCE_DIR)/driver && make KERNEL_LOCATION=$(KERNEL_VER)
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
