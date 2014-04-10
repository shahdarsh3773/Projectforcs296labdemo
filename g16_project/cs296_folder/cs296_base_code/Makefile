.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
FILE    = ./external/include/Box2D/Box2D.h
FILE2   = ./external/lib/libBox2D.a
######################################
# Project Name (generate executable with this name)
TARGET = cs296_16_exe

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/myobjs
BINDIR = $(PROJECT_ROOT)/mybins
LIBDIR = $(PROJECT_ROOT)/mylibs
DOCDIR = $(PROJECT_ROOT)/doc
EXTER_SRC=./external/src

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing -fPIC
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
LDFLAGS1 +=-pg -o3 -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
LDFLAGS+ =-pg -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib
CPPFLAGS1 =-pg -O3 -Wall -fno-strict-aliasing
CPPFLAGS1+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
CPPFLAGS2 =-pg -Wall -fno-strict-aliasing
CPPFLAGS2+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
OBJS1 := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%1.o)
OBJS2 := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%2.o)
SHARED_LIB=true
FILESLIB := callbacks.o  cs296_base.o  dominos.o render.o




.PHONY: all  setup doc distclean clean

all: setup exe makelib exelib 

setup:  
	@$(ECHO) "Setting up compilation..."
	@mkdir -p mylibs
	@mkdir -p myobjs
	@mkdir -p mybins
	echo "checking if BOX2D is installed"
	@ if [[ -f $(FILE) && -f $(FILE2) ]]; then \
	echo "BOX2D is installed"; \
	else \
	echo "Installing"; \
	cd $(EXTER_SRC); tar -zxf Box2D.tgz; \
	cd Box2D; \
	pwd; \
	mkdir -p build296; \
	cd build296; \
	cmake ../; \
	make; \
	make install; \
	fi

setup1:  
	@$(ECHO) "Setting up compilation..."
	@mkdir -p mylibs
	@mkdir -p myobjs
	@mkdir -p mybins
	echo "checking if BOX2D is installed"
	@ if [[ -f $(FILE) && -f $(FILE2) ]]; then \
	echo "BOX2D is installed"; \
	else \
	echo "Installing"; \
	cd $(EXTER_SRC); tar -zxf Box2D.tgz; \
	cd Box2D; \
	pwd; \
	mkdir -p build296; \
	cd build296; \
	cmake -DCMAKE_BUILD_TYPE=Release ../; \
	make; \
	make install; \
	fi

setup2:  
	@$(ECHO) "Setting up compilation..."
	@mkdir -p mylibs
	@mkdir -p myobjs
	@mkdir -p mybins
	echo "checking if BOX2D is installed"
	@ if [[ -f $(FILE) && -f $(FILE2) ]]; then \
	echo "BOX2D is installed"; \
	else \
	echo "Installing"; \
	cd $(EXTER_SRC); tar -zxf Box2D.tgz; \
	cd Box2D; \
	pwd; \
	mkdir -p build296; \
	cd build296; \
	cmake -DCMAKE_BUILD_TYPE=Debug ../; \
	make; \
	make install; \
	fi


exe: setup $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $(TARGET))"
	@$(CC) -o $(TARGET) $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@mv ./$(TARGET) ./mybins
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

exe1: setup $(OBJS1)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $(TARGET))"
	@$(CC) -pg -o3 -o $(TARGET) $(LDFLAGS) $(OBJS1) $(LIBS) 2> temp.log || touch temp.err
	@mv ./$(TARGET) ./mybins
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

exe2: setup $(OBJS2)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $(TARGET))"
	@$(CC) -pg -o $(TARGET) $(LDFLAGS) $(OBJS2) $(LIBS) 2> temp.log || touch temp.err
	@mv ./$(TARGET) ./mybins
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

$(OBJS1): $(OBJDIR)/%1.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS1) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

$(OBJS2): $(OBJDIR)/%2.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS2) -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

makelib: setup $(OBJS) 
	@ if [ $(SHARED_LIB) == true ]; then \
	echo "true"; \
	cd myobjs; \
	gcc -shared -Wl,-soname,libCS296test.so -o libCS296test.so $(FILESLIB) -lc; \
	mv libCS296test.so ../mylibs ; \
	else \
	cd myobjs; \
	ar rcs libCS296test.a $(FILESLIB); \
	mv libCS296test.a ../mylibs ; \
	echo "false"; \
	fi

exelib: makelib
	@ cd myobjs; \
	$(CC) -o cs296_16_exelib $(LDFLAGS) -L ../mylibs/ -L ../external/lib/ main.o -lCS296test $(LIBS)
	@ mv myobjs/cs296_16_exelib ./mybins

report : 
	 cd doc && pwd && pdflatex cs296_project_report.tex && bibtex cs296_project_report && pdflatex cs296_project_report.tex
	@$(RM) -rf ./doc/cs296_project_report.bib
	@$(RM) -rf ./doc/cs296_project_report.aux
	@$(RM) -rf ./doc/cs296_project_report.bbl
	@$(RM) -rf ./doc/cs296_project_report.blg
	@$(RM) -rf ./doc/cs296_project_report.log

plotPython: exe
	@ /opt/python3.3/bin/python3.3 ./scriptsPython/g16_gen_csv.py
	@ /opt/python3.3/bin/python3.3 ./scriptsPython/g16_gen_plots.py

profileReport: 
	@ /opt/python3.3/bin/python3.3 ./scriptsPython/g16_lab09_report.py 
		
dist: distclean
	@ cd .. && tar -cvzf Project.tar cs296_base_code

install: exe doc report
	@ cd .. && mkdir -p FinalProduct 
	@ cp ./mybins/cs296_16_exe ../FinalProduct/cs296_16_exe
	@ cp ./doc/cs296_project_report.pdf ../FinalProduct/cs296_project_report.pdf
	@ cd .. && cd FinalProduct && mkdir -p html
	@ cp -r ./doc/html/* ../FinalProduct/html/

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJDIR) *~ $(DEPS) $(SRCDIR)/*~
	@$(RM) -rf ./mybins/*
	@$(RM) -rf ./mylibs/* 
	@$(ECHO) "Done"
	@$(RM) -rf ./data/*
	@$(RM) -rf ./plots/*
	@$(RM) -rf ./gmon.out
	@$(RM) -rf ./doc/profileReport.html
	@$(RM) -rf ./doc/report.pdf
	

distclean: clean
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html
	@$(RM) -rf ./external/include/*
	@$(RM) -rf ./external/lib/*
	@$(RM) -rf ./mylibs
	@$(RM) -rf ./mybins
	@$(RM) -rf ./myobjs
	@$(RM) -rf ./external/src/Box2D
	@$(RM) -rf ./doc/x.log
	@$(RM) -rf ./doc/html/
	
debug_prof: distclean setup2 exe2
	@ ./mybins/cs296_16_exe 100000
	@ gprof ./mybins/cs296_16_exe gmon.out > ./data/g16_debug_prof.dat

release_prof: distclean setup1 exe1
	@ ./mybins/cs296_16_exe 100000
	@ gprof ./mybins/cs296_16_exe gmon.out > ./data/g16_release_prof.dat



	





        
