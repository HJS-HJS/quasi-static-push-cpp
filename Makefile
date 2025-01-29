# 컴파일러 설정
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wno-sign-compare -Iinclude -I/usr/include/eigen3

# 디렉토리 설정
SRC_DIR = src
TEST_DIR = test
BIN_DIR = bin

# 라이브러리 설정
LIBS = -lSDL2 -lSDL2_gfx  # SDL2_gfx 추가

# 소스 파일 설정
DIAGRAM_SRC    = $(wildcard $(SRC_DIR)/diagram/*.cpp)
SIMULATION_SRC = $(wildcard $(SRC_DIR)/simulation/*.cpp)
VIEWER_SRC     = $(wildcard $(SRC_DIR)/viewer/*.cpp)
SOLVER_SRC     = $(wildcard $(SRC_DIR)/solver/*.cpp)

# 테스트 파일 설정
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# 객체 파일 설정
DIAGRAM_OBJ    = $(DIAGRAM_SRC:.cpp=.o)
SIMULATION_OBJ = $(SIMULATION_SRC:.cpp=.o)
VIEWER_OBJ     = $(VIEWER_SRC:.cpp=.o)
SOLVER_OBJ     = $(SOLVER_SRC:.cpp=.o)
TEST_OBJ       = $(TEST_FILES:.cpp=.o)

# 실행 파일 설정
EXECUTABLES = $(TEST_FILES:$(TEST_DIR)/%.cpp=$(BIN_DIR)/%)

# 빌드 대상
all: $(EXECUTABLES)

# 각 실행 파일 생성
$(BIN_DIR)/%: $(TEST_DIR)/%.cpp $(DIAGRAM_OBJ) $(SIMULATION_OBJ) $(VIEWER_OBJ) $(SOLVER_OBJ)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

# 개별 객체 파일 생성 규칙
$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 클린업 규칙
clean:
	rm -f $(DIAGRAM_OBJ) $(SIMULATION_OBJ) $(VIEWER_OBJ) $(SOLVER_OBJ) $(BIN_DIR)/*
