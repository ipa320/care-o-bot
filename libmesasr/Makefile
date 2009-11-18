all: libmesasr

DPKG = libmesasr-dev-1.0.12-583.i386.deb
DPKG_URL = http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.12-583.i386.deb
SOURCE_DIR = libmesasr-deb/

libmesasr:
	-mkdir $(SOURCE_DIR)
	wget $(DPKG_URL) -O $(SOURCE_DIR)/$(DPKG)
	sudo dpkg -i $(SOURCE_DIR)/$(DPKG)
clean:
	rm -rf $(SOURCE_DIR)
wipe: clean
	-rm -rf $(SOURCE_DIR)
