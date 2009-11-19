all: libpcan

TARBALL = build/peak-linux-driver-6.11
TARBALL_URL = http://p103112.typo3server.info/fileadmin/media/linux/files/peak-linux-driver.6.11.tar.gz
SOURCE_DIR = build/peak-linux-driver-6.11
include $(shell rospack find mk)/download_unpack_build.mk

pycrypto: $(SOURCE_DIR)/unpacked
	-rm -rf src
	mkdir -p src
	cd $(SOURCE_DIR) && python setup.py build 
	mv $(SOURCE_DIR)/build/lib.*/Crypto src
	touch pycrypto
clean:
	rm -rf src pycrypto $(SOURCE_DIR)
wipe: clean
	-rm -rf build
