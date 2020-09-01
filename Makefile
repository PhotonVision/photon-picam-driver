# Pick up the appropriate files and make a runnable binary
OBJS=PicamJNI.o RaspiTex.o RaspiTexUtil.o RaspiCamControl.o RaspiHelpers.o vcsm_square.o TesterMain.o
SRC=TesterMain.cpp PicamJNI.cpp RaspiTex.c RaspiCamControl.c RaspiHelpers.c RaspiTexUtil.c vcsm_square.c

# We need these so that GCC vectorizes the loop that copies out every fourth pixel from VCSM
CFLAGS+=-O3 -mfpu=neon -ftree-vectorize -fPIC -g

# Looots of dependencies
LDFLAGS+=-lbrcmGLESv2 -lbrcmEGL -lbcm_host -lvcsm -lmmal -lmmal_core -lmmal_util -lm -ldl -lpthread -lstdc++
INCLUDES+=-I$(SDKSTAGE)/opt/vc/include/

# Include JNI files from the default JDK
JNI_INCLUDE=/usr/lib/jvm/java-11-openjdk-armhf/include \
    /usr/lib/jvm/java-11-openjdk-armhf/include/linux
INCLUDES+=$(foreach d, $(JNI_INCLUDE), -I$d)

# Add OpenCV library deps
CFLAGS+=`pkg-config --cflags opencv`
INCLUDES+=`pkg-config --libs opencv`
LDFLAGS+=`pkg-config --libs opencv`

include ../Makefile.include

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

tester: ${OBJS}
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ ${OBJS} $(LDFLAGS)

libpicam.so: ${OBJS}
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ -shared $(SRC) $(LDFLAGS) 

$(info ${OBJS})
