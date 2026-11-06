IMGUI_DIR := external/imgui
GLAD_DIR  := external/glad
GLM_DIR   := external/glm

CXX      := g++
CC       := gcc
CXXFLAGS := -std=c++17 -Wall -Wextra -g -O2 -MMD -MP
CFLAGS   := -O2 -MMD -MP
CPPFLAGS := -Isrc -isystem $(IMGUI_DIR) -isystem $(IMGUI_DIR)/backends \
            -isystem $(GLAD_DIR)/include -isystem $(GLM_DIR)
LDLIBS   := -lglfw -lGL -ldl -lpthread

SRC_DIR   := src
BUILD_DIR := build
OBJ_DIR   := $(BUILD_DIR)/obj
TARGET    := $(BUILD_DIR)/csg
TEST_BIN  := $(BUILD_DIR)/csg_tests

SRCS := $(shell find $(SRC_DIR) -name '*.cpp')
OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# CSG core (no GL/ImGui), shared by the app and the tests
CORE_OBJS := $(filter $(OBJ_DIR)/csg/%.o,$(OBJS))

IMGUI_SRCS := $(IMGUI_DIR)/imgui.cpp \
              $(IMGUI_DIR)/imgui_demo.cpp \
              $(IMGUI_DIR)/imgui_draw.cpp \
              $(IMGUI_DIR)/imgui_tables.cpp \
              $(IMGUI_DIR)/imgui_widgets.cpp \
              $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp \
              $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
IMGUI_OBJS := $(IMGUI_SRCS:$(IMGUI_DIR)/%.cpp=$(OBJ_DIR)/imgui/%.o)

GLAD_OBJ := $(OBJ_DIR)/glad/gl.o

TEST_SRCS := $(wildcard tests/*.cpp)
GPU_CHECK := $(BUILD_DIR)/gpu_check
GPU_OBJS  := $(OBJ_DIR)/tests/gpu/gpu_check.o $(filter $(OBJ_DIR)/render/%.o,$(OBJS)) $(CORE_OBJS) $(GLAD_OBJ)
TEST_OBJS := $(TEST_SRCS:tests/%.cpp=$(OBJ_DIR)/tests/%.o)

DEPS := $(OBJS:.o=.d) $(IMGUI_OBJS:.o=.d) $(GLAD_OBJ:.o=.d) $(TEST_OBJS:.o=.d) $(OBJ_DIR)/tests/gpu/gpu_check.d

.PHONY: all run test test-gpu clean

all: $(TARGET)

$(TARGET): $(OBJS) $(IMGUI_OBJS) $(GLAD_OBJ)
	$(CXX) $^ -o $@ $(LDLIBS)

$(TEST_BIN): $(TEST_OBJS) $(CORE_OBJS)
	$(CXX) $^ -o $@ -lpthread

$(GPU_CHECK): $(GPU_OBJS)
	$(CXX) $^ -o $@ $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/imgui/%.o: $(IMGUI_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/glad/%.o: $(GLAD_DIR)/src/%.c
	@mkdir -p $(@D)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(OBJ_DIR)/tests/%.o: tests/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

test: $(TEST_BIN)
	./$(TEST_BIN)

# Compares the GPU ray caster with the CPU classification (needs a display).
test-gpu: $(GPU_CHECK)
	./$(GPU_CHECK)

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
