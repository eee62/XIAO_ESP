# Put esp-dl's headers on the include path for src/.
#
# pioarduino compiles managed IDF components through CMake, but src/*.cpp is
# compiled by the Arduino/SCons side, which does not inherit the components'
# INCLUDE_DIRS. Without this, main.cpp cannot find dl_image_jpeg.hpp or
# pedestrian_detect.hpp even though the component builds fine.
#
# The list mirrors include_dirs in managed_components/espressif__esp-dl/
# CMakeLists.txt (plus the ESP32-S3 ISA dirs it appends for that target).
# Paths are added unconditionally: on a clean build this script runs before
# the component manager has fetched anything, so existence cannot be checked
# here. A stale entry is a harmless no-op; a missing header fails the compile
# loudly, which is what we want.

Import("env")
import os

project_dir = env.subst("$PROJECT_DIR")
managed = os.path.join(project_dir, "managed_components")

esp_dl = os.path.join(managed, "espressif__esp-dl")
esp_dl_subdirs = [
    "dl",
    "dl/tool/include",
    "dl/tensor/include",
    "dl/base",
    "dl/base/isa",
    "dl/base/isa/xtensa",   # ESP32-S3
    "dl/base/isa/tie728",   # ESP32-S3
    "dl/math/include",
    "dl/model/include",
    "dl/module/include",
    "fbs_loader/include",
    "vision/detect",
    "vision/image",
    "vision/image/isa",
    "vision/recognition",
    "vision/classification",
    "audio/common",
    "audio/speech_features",
]

includes = [os.path.join(esp_dl, d) for d in esp_dl_subdirs]

# esp_new_jpeg, the decoder behind esp-dl's sw_decode_jpeg(). main.cpp calls
# it directly for its scaled decode, which sw_decode_jpeg() does not offer.
# It is esp-dl's own dependency, so it is fetched whenever esp-dl is.
includes.append(os.path.join(managed, "espressif__esp_new_jpeg", "include"))

# Model components keep their header at the component root.
for model in ("espressif__pedestrian_detect", "espressif__human_face_detect"):
    includes.append(os.path.join(managed, model))

env.Append(CPPPATH=includes)
