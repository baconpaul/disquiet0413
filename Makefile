dq0413:	disquiet0413.cpp
	clang++ -O3 -std=c++14 -Werror -o dq0413 disquiet0413.cpp -lsndfile

run:	dq0413
	./dq0413
	afplay disquiet0413.wav

clean:
	rm -f ./dq0413
	rm -f ./disquiet0413.wav
