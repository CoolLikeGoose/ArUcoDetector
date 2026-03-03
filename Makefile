UNITY_PLUGIN_DIR=C:\Unity\BacProject\Assets\Plugins\Android\arm64-v8a
OUTPUT_SO=.\build\libaruco.so

all: compile copy

compile:
	@echo Building Android plugin...
	android_build.bat

copy:
	@echo Deleteing previous plugin...
	@if exist "$(UNITY_PLUGIN_DIR)\libaruco.so" del "$(UNITY_PLUGIN_DIR)\libaruco.so"
	@if exist "$(UNITY_PLUGIN_DIR)\libaruco.so.meta" del "$(UNITY_PLUGIN_DIR)\libaruco.so.meta"
	@echo Copying plugin to Unity...
	@copy "$(OUTPUT_SO)" "$(UNITY_PLUGIN_DIR)"
	@echo Done!

clean:
	@echo Cleaning build...
	@rmdir /s /q build

# win:
# 	cl test.cpp /I C:\opencv\build\include /link /LIBPATH:C:\opencv\build\x64\vc16\lib opencv_world4120.lib
