.PHONY: build

BIN_NAME="controller"
CFLAGS= -o $(BIN_NAME) -Wall -lSDL2
CC= gcc
CFILES= main.c

build:
	echo "> building app (debug)"
	$(CC) $(CFILES) $(CFLAGS) -gdwarf-2 -O0
	echo "> app built (debug)"

.PHONY: optbuild
optbuild:
	echo "> building app (speed optimized)"
	$(CC) $(CFILES) $(CFLAGS) -O3
	echo "> app built (speed optimized)"

.PHONY: run
run: build
	./$(BIN_NAME)

.PHONY: optrun
optrun: optbuild
	./$(BIN_NAME)

.PHONY: clean
clean:
	rm -f $(BIN_NAME)
