all:
	mkdir -p build
	pandoc \
	  -f markdown+implicit_header_references \
	  -t html5 \
	  --output build/index.html \
	  --template templates/default.html5 \
	  --css ../templates/book.css \
	  --toc \
	  --mathjax \
	  --number-sections \
	  title.txt \
	  book.md
	cp -R img build/


watch:
	while true; do inotifywait -r -e close_write .; make; done

deploy:
	scp -r build/* rmarko@48.io:public_html/book/
	scp -r templates/ rmarko@48.io:public_html/
