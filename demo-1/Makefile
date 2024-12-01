# 定义编译器
CC = arm-none-eabi-gcc

# 目标
TARGET = my_test

# 列出所有源文件
SRCS = $(wildcard src/*.c) 
 
# 将源文件转换为目标文件列表
OBJS = $(SRCS:.c=.o)
  
# 默认目标
all: $(TARGET)

# 生成文件
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET)
 

