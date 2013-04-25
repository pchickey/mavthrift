

GENERATOR_DIR   =./thrift-idl-generator/generator
GENERATOR_EXE   = python $(GENERATOR_DIR)/generate.py
MESSAGE_DEF_DIR =./thrift-idl-generator/message_definitions

MAVTHRIFT_DIR      =./generated-thrift
MAVTHRIFT_CPP_DIR  =$(MAVTHRIFT_DIR)/gen-cpp
LIBMAVTHRIFT = $(MAVTHRIFT_DIR)/libmavthrift.a

MAVTHRIFT_THRIFT_FILES = $(addprefix $(MAVTHRIFT_DIR)/,common.thrift ardupilotmega.thrift)

# RULES ########################################################################

all: $(LIBMAVTHRIFT)

$(LIBMAVTHRIFT): $(MAVTHRIFT_THRIFT_FILES) 
	$(MAKE) -C $(MAVTHRIFT_DIR)

.PHONY:
regenerate-mavlink-thrift: $(MAVTHRIFT_THRIFT_FILES)
	$(foreach THRIFT_FILE,$(MAVTHRIFT_THRIFT_FILES), \
		thrift -gen cpp -o $(MAVTHRIFT_DIR) $(THRIFT_FILE); \
		)

$(MAVTHRIFT_THRIFT_FILES): $(MESSAGE_DEF_DIR)/ardupilotmega.xml
$(MAVTHRIFT_THRIFT_FILES): $(MESSAGE_DEF_DIR)/common.xml
	$(GENERATOR_EXE) -o $(MAVTHRIFT_DIR) \
		$(MESSAGE_DEF_DIR)/ardupilotmega.xml

clean:
	-rm -rf $(MAVTHRIFT_CPP_DIR)/*.o
	-rm -rf $(LIBMAVTHRIFT)

veryclean: clean
	-rm -rf $(MAVTHRIFT_CPP_DIR)
	-rm $(MAVTHRIFT_THRIFT_FILES)
