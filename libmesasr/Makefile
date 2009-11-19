all: libmesasr

DPKG = libmesasr-dev-1.0.12-583.i386.deb
DPKG_URL = http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.12-583.i386.deb
SOURCE_DIR = build

libmesasr:
	# get zip-file
	mkdir -p $(SOURCE_DIR)
	cd $(SOURCE_DIR) && wget $(DPKG_URL) -O $(DPKG)
	# install package
	sudo dpkg -i $(SOURCE_DIR)/$(DPKG)
clean:
	rm -rf $(SOURCE_DIR)
wipe: clean
	-rm -rf $(SOURCE_DIR)

