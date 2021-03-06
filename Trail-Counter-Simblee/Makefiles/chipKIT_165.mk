#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2016
# http://embedxcode.weebly.com
# All rights reserved
#
#
# Last update: Oct 17, 2016 release 5.3.4



# chipKIT specifics
# ----------------------------------
# OPT_SYSTEM_INTERNAL is defined in main.cpp but used in wiring.h
#
PLATFORM         := chipKIT
BUILD_CORE       := pic32
APPLICATION_PATH := $(CHIPKIT_PATH)
PLATFORM_VERSION := $(CHIPKIT_RELEASE) for Arduino 1.6.12

HARDWARE_PATH     = $(APPLICATION_PATH)/hardware/pic32/$(CHIPKIT_RELEASE)
#mp001             = $(shell cat $(APPLICATION_PATH)/lib/version.txt | cut -d- -f1 | sed 's/^0*//')
PLATFORM_TAG      = ARDUINO=10612 MPIDE=$(CHIPKIT_RELEASE) IDE=Arduino MPIDEVER=16777998 EMBEDXCODE=$(RELEASE_NOW) ARDUINO_ARCH_PIC32

TOOL_CHAIN_PATH   = $(CHIPKIT_PATH)/tools/pic32-tools/$(PIC32_GCC_RELEASE)
OTHER_TOOLS_PATH  = $(CHIPKIT_PATH)/tools/pic32prog/$(PIC32_PROG_RELEASE)

UPLOADER          = pic32prog
UPLOADER_PATH     = $(OTHER_TOOLS_PATH)
UPLOADER_EXEC     = $(UPLOADER_PATH)/$(UPLOADER)

APP_TOOLS_PATH   := $(TOOL_CHAIN_PATH)/bin
CORE_LIB_PATH    := $(HARDWARE_PATH)/cores/$(BUILD_CORE)
APP_LIB_PATH     := $(HARDWARE_PATH)/libraries

BOARDS_TXT       := $(HARDWARE_PATH)/boards.txt
ifeq ($(call PARSE_FILE,$(BOARD_TAG),name,$(BOARDS_TXT)),)
    BOARDS_TXT   := $(shell grep -rnwls $(HARDWARE_PATH)/variants -e '$(BOARD_TAG).name')
endif


# Sketchbook/Libraries path
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
ifeq ($(wildcard $(USER_LIBRARY_DIR)/Arduino15/preferences.txt),)
    $(error Error: run Mpide once and define the sketchbook path)
endif

ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    SKETCHBOOK_DIR = $(shell grep sketchbook.path $(USER_LIBRARY_DIR)/Arduino15/preferences.txt | cut -d = -f 2)
endif
ifeq ($(wildcard $(SKETCHBOOK_DIR)),)
    $(error Error: sketchbook path not found)
endif
USER_LIB_PATH  = $(wildcard $(SKETCHBOOK_DIR)/?ibraries)

# USER sources
# wildcard required for ~ management
# ?ibraries required for libraries and Libraries
#
# Network and WiFi libraries need to be compiled on a given order!
#
ifndef USER_LIBS_LIST
    ew001               = $(realpath $(sort $(dir $(wildcard $(USER_LIB_PATH)/*/*.h)))) # */
    USER_LIBS_LIST      = $(subst $(USER_LIB_PATH)/,,$(filter-out $(EXCLUDE_LIST),$(ew001)))
endif
ew002                   = WiFiShieldOrPmodWiFi_G DNETcK DWIFIcK

# Libraries for WiFi
# Success if included in the following order
#
# MPIDE 1.5  = WiFiShieldOrPmodWiFi_G DNETcK DWIFIcK
# MPIDE 0023 = MRF24G DEIPcK DEWFcK HTTPServer

ifneq ($(USER_LIBS_LIST),0)
    ifneq ($(filter $(ew002),$(USER_LIBS_LIST)),)
        ew003           = $(filter-out $(ew002),$(USER_LIBS_LIST))
        ew004           = $(filter $(ew002),$(USER_LIBS_LIST)) $(addsuffix /utility,$(filter $(ew002),$(USER_LIBS_LIST)))
        ew007          += $(addprefix $(USER_LIB_PATH)/,$(ew004))
        USER_LIBS_LOCK  = 1
    else
        ew003           = $(USER_LIBS_LIST)
    endif
    ew005               = $(patsubst %,$(USER_LIB_PATH)/%,$(ew003))

    ew006               = $(foreach dir,$(ew005),$(shell find $(dir) -type d))
    USER_LIBS           = $(ew006) $(ew007)
    USER_LIB_CPP_SRC    = $(wildcard $(patsubst %,%/*.cpp,$(USER_LIBS))) # */
    USER_LIB_C_SRC      = $(wildcard $(patsubst %,%/*.c,$(USER_LIBS))) # */

    USER_OBJS           = $(patsubst $(USER_LIB_PATH)/%.cpp,$(OBJDIR)/user/%.cpp.o,$(USER_LIB_CPP_SRC))
    USER_OBJS          += $(patsubst $(USER_LIB_PATH)/%.c,$(OBJDIR)/user/%.c.o,$(USER_LIB_C_SRC))
endif

REMOTE_OBJS = $(sort $(CORE_OBJS) $(BUILD_CORE_OBJS) $(APP_LIB_OBJS) $(BUILD_APP_LIB_OBJS) $(VARIANT_OBJS)) $(USER_OBJS)


# Rules for making a c++ file from the main sketch (.pde)
#
PDEHEADER      = \\\#include \"WProgram.h\"  


# Tool-chain names
#
CC      = $(APP_TOOLS_PATH)/pic32-gcc
CXX     = $(APP_TOOLS_PATH)/pic32-g++
#AS      = $(APP_TOOLS_PATH)/pic32-g++
AR      = $(APP_TOOLS_PATH)/pic32-ar
OBJDUMP = $(APP_TOOLS_PATH)/pic32-objdump
OBJCOPY = $(APP_TOOLS_PATH)/pic32-objcopy
SIZE    = $(APP_TOOLS_PATH)/pic32-size
NM      = $(APP_TOOLS_PATH)/pic32-nm
# ~
MDB     = $(APPLICATIONS_PATH)/microchip/mplabx/mplab_ide.app/Contents/Resources/mplab_ide/bin/mdb.sh
# ~~

# Release 1.0.1 = board ; release 1.1.0 = build.board
BOARD    = $(call PARSE_BOARD,$(BOARD_TAG),board)
ifeq ($(BOARD),)
    BOARD    = $(call PARSE_BOARD,$(BOARD_TAG),build.board)
endif

LDSCRIPT = $(call PARSE_BOARD,$(BOARD_TAG),ldscript)
LDCOMMON = $(call PARSE_BOARD,$(BOARD_TAG),ldcommon)
ifeq ($(LDCOMMON),)
    LDCOMMON := chipKIT-application-COMMON.ld
endif
VARIANT  = $(call PARSE_BOARD,$(BOARD_TAG),build.variant)
#VARIANT_PATH = $(APPLICATION_PATH)/hardware/pic32/variants/$(VARIANT)
VARIANT_PATH = $(HARDWARE_PATH)/variants/$(VARIANT)

# Add .S files required by MPIDE release 0023-macosx-20130715
#
CORE_AS_SRCS    = $(wildcard $(CORE_LIB_PATH)/*.S) # */
mp002           = $(patsubst %.S,%.S.o,$(filter %.S, $(CORE_AS_SRCS)))
FIRST_O_IN_A    = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(mp002))


# Two locations for Arduino libraries
#
VARIANT_C_SRCS    = $(wildcard $(VARIANT_PATH)/*.c) # */
VARIANT_OBJ_FILES = $(VARIANT_C_SRCS:.c=.c.o)
VARIANT_OBJS      = $(patsubst $(APPLICATION_PATH)/%,$(OBJDIR)/%,$(VARIANT_OBJ_FILES))


# ~
ifeq ($(BOARD_PORT),pgm)
    OPTIMISATION   ?= -O0
    NO_SERIAL_CONSOLE = 1
else
    OPTIMISATION   ?= -O2
endif
# ~~

MCU_FLAG_NAME    = mprocessor
MCU              = $(call PARSE_BOARD,$(BOARD_TAG),build.mcu)


INCLUDE_PATH    = $(CORE_LIB_PATH) $(APP_LIB_PATH) $(VARIANT_PATH) $(HARDWARE_PATH)
INCLUDE_PATH   += $(sort $(dir $(APP_LIB_CPP_SRC) $(APP_LIB_C_SRC) $(APP_LIB_H_SRC)))
INCLUDE_PATH   += $(sort $(dir $(BUILD_APP_LIB_CPP_SRC) $(BUILD_APP_LIB_C_SRC)))
INCLUDE_PATH   += $(OBJDIR)



# Flags for gcc, g++ and linker
# ----------------------------------
#
# Common CPPFLAGS for gcc, g++, assembler and linker
#
CPPFLAGS     = $(OPTIMISATION)
CPPFLAGS    += -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
CPPFLAGS    += $(addprefix -D, $(PLATFORM_TAG)) -D$(BOARD)
CPPFLAGS    += $(addprefix -I, $(INCLUDE_PATH))

# Specific CFLAGS for gcc only
# gcc uses CPPFLAGS and CFLAGS
#
#CFLAGS       = -g -ffunction-sections -fdata-sections -G1024 -mdebugger -Wcast-align -fno-short-double
m101         = $(call PARSE_BOARD,$(BOARD_TAG),compiler.c.flags)
ifeq ($(m101),)
    m101    := -O2::-c::-mno-smart-io::-w::-ffunction-sections::-fdata-sections::-G1024::-g::-mdebugger::-Wcast-align::-fno-short-double
endif
m102         = $(shell echo '$(m101)' | sed 's/::/ /g')
CFLAGS       = $(filter-out -O%,$(m102))

# Specific CXXFLAGS for g++ only
# g++ uses CPPFLAGS and CXXFLAGS
#
#CXXFLAGS     = -g -fno-exceptions -ffunction-sections -fdata-sections -G1024 -mdebugger -Wcast-align -fno-short-double
m201         = $(call PARSE_BOARD,$(BOARD_TAG),compiler.cpp.flags)
ifeq ($(m201),)
    m201    := -O2::-c::-mno-smart-io::-w::-ffunction-sections::-fdata-sections::-G1024::-g::-mdebugger::-Wcast-align::-fno-short-double
endif
m202         = $(shell echo '$(m201)' | sed 's/::/ /g')
CXXFLAGS     = $(filter-out -O%,$(m202)) -ftoplevel-reorder

# Specific ASFLAGS for gcc assembler only
# gcc assembler uses CPPFLAGS and ASFLAGS
#
ASFLAGS      = -g1 -Wa,--gdwarf-2

# Specific LDFLAGS for linker only
# linker uses CPPFLAGS and LDFLAGS
# Thanks to ricklon for spotting the issue on linking!
# -T<space>ldscript and not -Tldscript
#
# chipKIT-application-COMMON.ld added by MPIDE release 0023-macosx-20130715
LDFLAGS    = $(OPTIMISATION) -Wl,--gc-sections -$(MCU_FLAG_NAME)=$(MCU) -DF_CPU=$(F_CPU)
LDFLAGS   += -T $(VARIANT_PATH)/$(LDSCRIPT) -T $(CORE_LIB_PATH)/$(LDCOMMON)
LDFLAGS   += -mdebugger -mno-peripheral-libs -nostartfiles


# Commands
# ----------------------------------
# Link command
# compatible with MPIDE release 0023-macosx-20130715
#
COMMAND_LINK    = $(CXX) $(LDFLAGS) $(OUT_PREPOSITION)$@ $(LOCAL_OBJS) $(LOCAL_ARCHIVES) $(TARGET_A) -L$(OBJDIR) -lm

COMMAND_UPLOAD  = $(UPLOADER_EXEC) -d $(USED_SERIAL_PORT) $(TARGET_HEX)
