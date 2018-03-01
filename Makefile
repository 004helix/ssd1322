CC = gcc
LD = gcc
CFLAGS = -Wall -Werror
LDFLAGS = 

EXE = ssd1322

$(EXE): $(EXE).c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(EXE)
