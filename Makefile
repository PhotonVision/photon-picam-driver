# Pick up the appropriate files and make a runnable binary
OBJS=PicamJNI.o RaspiTex.o RaspiTexUtil.o RaspiCamControl.o RaspiHelpers.o ThresholdingShader.o TesterMain.o
SRC=TesterMain.cpp PicamJNI.cpp RaspiTex.c RaspiCamControl.c RaspiHelpers.c RaspiTexUtil.c ThresholdingShader.c

# We need these so that GCC vectorizes the loop that copies out every fourth pixel from VCSM
CFLAGS+=-Ofast -mfpu=neon -ftree-vectorize -fPIC -Wall -Werror -Wno-unknown-warning-option -Wno-unused-command-line-argument -g -ggdb #-fsanitize=address -fsanitize=undefined
CXXFLAGS+=-std=c++17

# Looots of dependencies
LDFLAGS+=-ltbb -lbrcmGLESv2 -lbrcmEGL -lbcm_host -lvcsm -lmmal -lmmal_core -lmmal_util -lm -ldl -lpthread -lstdc++# -lasan -lubsan
INCLUDES+=-I$(SDKSTAGE)/opt/vc/include/

# Include JNI files from the default JDK
JNI_INCLUDE=/usr/lib/jvm/java-11-openjdk-armhf/include \
    /usr/lib/jvm/java-11-openjdk-armhf/include/linux
INCLUDES+=$(foreach d, $(JNI_INCLUDE), -I$d)

# Add OpenCV library deps
CFLAGS+=`pkg-config --cflags opencv`
INCLUDES+=`pkg-config --cflags-only-I opencv`#-Wl,--whole-archive,-Bstatic -lopencv_core -Wl,--no-whole-archive,-Bdynamic # Note that -Wl,-B only works on the GNU linker
LDFLAGS+=-Wl,--whole-archive,-Bstatic -lopencv_core -lz -Wl,--no-whole-archive,-Bdynamic # Note that -Wl,-B only works on the GNU linker


include ../Makefile.include

all: libpicam.so $(LIB)

# Meaning: 	$@ = file being generated (the .o file)
# 			$< = the first prereq (the .c file)
#			% = wildcard

%.o: %.c
	@rm -f $@ 
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

%.o: %.cpp
	@rm -f $@ 
	$(CXX) $(CFLAGS) $(CXXFLAGS) $(INCLUDES) -c $< -o $@ -Wno-deprecated-declarations

tester: ${OBJS}
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ ${OBJS} $(LDFLAGS)

libpicam.so: ${OBJS}
	$(CC) -shared $(INCLUDES) -o $@ ${OBJS} $(LDFLAGS)

$(info ${OBJS})
