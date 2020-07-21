OBJS=triangle.o video.o models.o PicamJNI.o

SRC=triangle.c video.c models.c PicamJNI.cpp

LDFLAGS+=-lilclient

CFLAGS+=-DSTANDALONE -D__STDC_CONSTANT_MACROS -D__STDC_LIMIT_MACROS -DTARGET_POSIX -D_LINUX -fPIC -DPIC -D_REENTRANT -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -U_FORTIFY_SOURCE -Wall -g -DHAVE_LIBOPENMAX=2 -DOMX -DOMX_SKIP64BIT -ftree-vectorize -pipe -DUSE_EXTERNAL_OMX -DHAVE_LIBBCM_HOST -DUSE_EXTERNAL_LIBBCM_HOST -DUSE_VCHIQ_ARM -Wno-psabi

LDFLAGS+=-L$(SDKSTAGE)/opt/vc/lib/ -lbrcmGLESv2 -lbrcmEGL -lopenmaxil -lbcm_host -lvcos -lvchiq_arm -lpthread -lrt -lm -L$(SDKSTAGE)/opt/vc/src/hello_pi/libs/ilclient -L$(SDKSTAGE)/opt/vc/src/hello_pi/libs/vgfont -L$(SDKSTAGE)/opt/vc/src/hello_pi/libs/revision

INCLUDES+=-I$(SDKSTAGE)/opt/vc/include/ -I$(SDKSTAGE)/opt/vc/include/interface/vcos/pthreads -I$(SDKSTAGE)/opt/vc/include/interface/vmcs_host/linux -I./ -I$(SDKSTAGE)/opt/vc/src/hello_pi/libs/ilclient -I$(SDKSTAGE)/opt/vc/src/hello_pi/libs/vgfont -I$(SDKSTAGE)/opt/vc/src/hello_pi/libs/revision -I$(SDKSTAGE)/usr/lib/jvm/java-11-openjdk-armhf/include/ -I$(SDKSTAGE)/usr/lib/jvm/java-11-openjdk-armhf/include/linux/

# Include JNI files from the default JDK
JNI_INCLUDE=/usr/lib/jvm/java-11-openjdk-armhf/include \
    /usr/lib/jvm/java-11-openjdk-armhf/include/linux
INCLUDES+=$(foreach d, $(JNI_INCLUDE), -I$d)

# Add OpenCV library deps
CFLAGS+=`pkg-config --cflags opencv`
INCLUDES+=`pkg-config --libs opencv`

all: libpicam.so $(LIB)

# Meaning: 	$@ = file being generated (the .o file)
# 			$< = the first prereq (the .c file)
#			% = wildcard

%.o: %.c
	@rm -f $@ 
	$(CC) $(CFLAGS) $(INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

%.o: %.cpp
	@rm -f $@ 
	$(CXX) $(CFLAGS) $(INCLUDES) -g -c $< -o $@ -Wno-deprecated-declarations

libpicam.so: ${OBJS}
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -shared $(SRC) $(LDFLAGS) 

clean:
	for i in $(OBJS); do (if test -e "$$i"; then ( rm $$i ); fi ); done
	@rm -f $(BIN) $(LIB)
	@rm -f *.o *~ 




# VERSION=2.0.1
# LIBRARY=picam-$VERSION.so
# JNI_INCLUDE=/usr/lib/jvm/java-11-openjdk-armhf/include
# JNI_LIB=/usr/lib/jvm/java-11-openjdk-armhf/lib
# PI_INCLUDE=/opt/vc/include
# PI_LIB=/opt/vc/lib
# LDFLAGS="-lc -lmmal -lmmal_core -lmmal_util"
# SRC="uk_co_caprica_picam_Camera.c Camera.c Configuration.c Defaults.c Encoder.c Port.c"
# gcc -I"$JNI_INCLUDE" -I"$JNI_INCLUDE/linux" -I"$PI_INCLUDE" -L"$PI_LIB" -o $LIBRARY -shared -Wl,-soname,$LIBRARY $SRC $LDFLAGS