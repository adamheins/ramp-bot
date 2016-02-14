CC=gcc

compile: clean
	@mkdir bin
	@python test/clar/generate.py test
	@$(CC) -std=c99 -I. src/*.c test/*.c test/clar/clar.c -o bin/test

test: compile
	./bin/test

clean:
	@rm -rf bin
