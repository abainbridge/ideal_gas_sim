src_dir=../../src
obj_dir=obj
deadfrog_lib_dir=../../../../deadfrog-lib
inc_dirs=-I $(deadfrog_lib_dir)/src
lib_dirs=-L $(deadfrog_lib_dir)/build/linux

cxxflags=-MMD -g -march=native -Wno-unused-result -fno-strict-aliasing -Ofast -flto

cpp_files_raw=\
	main.cpp \
	particles.cpp \
	world.cpp
cpp_files=$(addprefix $(src_dir)/,$(cpp_files_raw))
o_files=$(patsubst $(src_dir)/%.cpp,$(obj_dir)/%.o,$(cpp_files))
d_files=$(patsubst %.o,%.d,$(o_files))

ideal_gas_sim: $(obj_dir) $(o_files)
	g++ $(cxxflags) $(inc_dirs) $(o_files) -o $@ $(lib_dirs) -l deadfrog

# Tell Make to pay attention to the dependency files.
-include $(d_files)

$(obj_dir)/%.o: $(src_dir)/%.cpp Makefile
	g++ $(cxxflags) $(inc_dirs) $< -c -o $@

$(obj_dir)/main.o: $(src_dir)/main.cpp Makefile
	g++ $(cxxflags) $(inc_dirs) $< -c -o $@

$(obj_dir):
	mkdir -p $(obj_dir)

clean:
	rm -rf $(obj_dir) ideal_gas_sim
