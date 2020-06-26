# makefile is based on https://spin.atomicobject.com/2016/08/26/makefile-c-projects/
# original makefile author: Job Vranish
# - out-of-source builds (object files get dumped in a separate directory from the source)
# - automatic (and accurate!) header dependencies
# - automatic determination of list of object/source files
# - automatic generation of include directory flags

TARGET_EXEC ?= hardnes

BUILD_DIR ?= ./build
SRC_DIRS ?= ./src

LIBS := -lSDL2

SRCS := $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c -or -name *.s)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

INC_DIRS := $(shell find $(SRC_DIRS) -type d)
INC_FLAGS := $(addprefix -I,$(INC_DIRS))

CPPFLAGS ?= $(INC_FLAGS) -MMD -MP

$(TARGET_EXEC): $(OBJS)
	$(CC) $(OBJS) $(LIBS) -o $@ $(LDFLAGS)

# assembly
$(BUILD_DIR)/%.s.o: %.s
	$(MKDIR_P) $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.c.o: %.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	$(MKDIR_P) $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean

clean:
	$(RM) $(TARGET_EXEC)
	$(RM) -r $(BUILD_DIR)

-include $(DEPS)

MKDIR_P ?= mkdir -p


#all: hardnes
#
#hardnes: *.o
#	gcc -o hardnes *.o
#
#%.d: src/%.c
#	gcc -M $< > $@
#
#%.o: src/%.c %.d
#	gcc -c $< -o $@
#
#clean:
#	rm *.o hardnes
#
