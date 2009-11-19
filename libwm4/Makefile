all: libwm4

ZIPFILE = WildMagic4p9.zip
ZIPFILE_URL = http://www.geometrictools.com/Downloads/WildMagic4p9.zip
SOURCE_DIR = libwm4-zip
BUILD_DIR = build

libwm4:
	mkdir $(SOURCE_DIR) -p
#	wget $(ZIPFILE_URL) -O $(SOURCE_DIR)/$(ZIPFILE)
	mkdir $(BUILD_DIR) -p
	cd $(BUILD_DIR) && unzip -n ../$(SOURCE_DIR)/$(ZIPFILE)
	cd $(BUILD_DIR)/GeometricTools/WildMagic4 && make -f makefile.wm4 CFG=Release SYS=linux GRF=OpenGL
clean:
	rm -rf $(SOURCE_DIR)
wipe: clean
	-rm -rf build
