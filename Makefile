all: bmi270_test

bmi270_test: fifo_watermark_header_mode.c bmi2.c bmi270.c
	gcc -Wall fifo_watermark_header_mode.c bmi2.c bmi270.c -o bmi270_test
