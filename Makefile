lint:
	cpplint \
		--extensions=ino \
		--linelength=120 \
		--filter=-legal/copyright,-readability/todo \
		*.ino
