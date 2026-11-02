LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_CPP_EXTENSION := .cpp .cc
LOCAL_MODULE := BetaRealismAction
LOCAL_SRC_FILES := main.cpp helmet_system.cpp npc_rescue_system.cpp crash_response_system.cpp rain_umbrella_system.cpp truck_air_brake_system.cpp beach_life_system.cpp blood_stain_system.cpp mod/logger.cpp mod/config.cpp
LOCAL_CXXFLAGS += -O2 -DNDEBUG -std=c++17
LOCAL_CPPFLAGS += -std=c++17
LOCAL_C_INCLUDES += $(LOCAL_PATH) $(LOCAL_PATH)/aml-psdk
LOCAL_LDLIBS += -llog -lOpenSLES
include $(BUILD_SHARED_LIBRARY)
