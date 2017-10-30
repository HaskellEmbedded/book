all:
	mkdir -p build
	pandoc \
	  -f markdown \
	  -t html5 \
	  --output build/index.html \
	  --template templates/default.html5 \
	  --css ../templates/book.css \
	  --toc \
	  --mathjax \
	  --number-sections \
	  title.txt \
	  book.md


watch-build:
	while true; do inotifywait -r -e close_write .; make; done
