clear
clear
gcc -o Minecraft ./platform_layer.cpp -Wno-deprecated-declarations -Wno-c++11-compat-deprecated-writable-strings -rpath /Library/Frameworks -I/Library/Frameworks/SDL2.framework/Headers -I/Library/Frameworks/SDL2_image.framework/Headers -F/Library/Frameworks -framework OpenGL -framework SDL2 -framework SDL2_image