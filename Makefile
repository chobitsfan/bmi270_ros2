all: bmi270_ros2

bmi270_ros2: fifo_watermark_header_mode.c bmi2.c bmi270.c
	gcc -Wall fifo_watermark_header_mode.c bmi2.c bmi270.c -o bmi270_ros2
