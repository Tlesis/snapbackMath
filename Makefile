CC=g++
CFLAGS=-Wall -g -std=c++20 -pedantic -Wextra -Wshadow
NAME=snapback
FILES=main.o

all: clean main run

# Compiles object files into one binary
main: $(FILES)
	$(CC) $(CFLAGS) $(FILES) -o $(NAME).out

# Makes .o object files
%.o: %.cpp
	$(CC) -O2 $(CFLAGS) -c $^

# Removes Compiled files
clean:
	@echo Cleaning. . .
	rm -f $(NAME).out *.o

# Compiles the project & runs the binary
run: clean main
	./$(NAME).out