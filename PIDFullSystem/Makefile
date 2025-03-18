CC = gcc
CFLAGS = -Wall -O2 -I simulation
LDFLAGS = -lm
SRC = src/main.c simulation/simulation.c
BIN = line_follower.exe

all: build plot

build:
    $(CC) $(CFLAGS) $(SRC) -o $(BIN) $(LDFLAGS)

run:
    $(BIN)

plot:
    python scripts/plot_results.py

clean:
    del $(BIN) sim_data.csv simulation_results.png