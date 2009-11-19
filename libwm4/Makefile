all: libwm4

ZIPFILE = WildMagic4p9.zip
ZIPFILE_URL = http://www.geometrictools.com/Downloads/WildMagic4p9.zip
SOURCE_DIR = build

libwm4:
	# get zip-file
	mkdir -p $(SOURCE_DIR)
	cd $(SOURCE_DIR) && wget $(ZIPFILE_URL) -N
	# unzip
	cd $(SOURCE_DIR); unzip -n $(ZIPFILE)
	# make
	cd $(SOURCE_DIR)/GeometricTools/WildMagic4; make -f makefile.wm4 CFG=Release SYS=linux GRF=OpenGL
	# move lib and header to export path (src)
	mkdir -p src
	mkdir -p src/lib
	mkdir -p src/include
	cp $(SOURCE_DIR)/GeometricTools/WildMagic4/SDK/Library/Release/lib* src/lib
	cp $(SOURCE_DIR)/GeometricTools/WildMagic4/SDK/Include/* src/include
clean:
	rm -rf src $(SOURCE_DIR)
wipe: clean
	-rm -rf $(SOURCE_DIR)
	
