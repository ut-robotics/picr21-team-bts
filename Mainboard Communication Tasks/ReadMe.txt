1)Install phtread library
Type following in terminal: sudo apt-get install libpthread-stubs0-dev

2)Install nscurses library
Type following in terminal: sudo apt-get install libncurses5-dev

3) Compile (con can be replaced with understandable name)
	gcc conBot.c -o con -lncurses -lpthread
4) Run (con can be replaced with understandable name)
	./con
