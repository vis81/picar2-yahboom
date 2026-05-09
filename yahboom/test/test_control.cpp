#include <gtest/gtest.h>

#include <iostream>

#include <errno.h>
#include <math.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#define PORTNAME "/dev/ttyUSB0"
#define BAUD_RATE B115200

#define READ_BUF_SIZE 64

#define CMD_SERVO_SET "$w%d:%d\n"
#define CMD_SERVO_GET "$r%d\n"

#define CMD_MOTOR_READ_POS "$p0\n$p1\n"
#define CMD_MOTOR_SET_SPEED "$s0:%d\n$s1:%d\n"
#define CMD_MOTOR_SET_THROTTLE "$t0:%d,%d\n$t1:%d,%d\n"
#define CMD_ECHO_PREFIX "$e:"
#define RESP_OK "@0\n"
#define RESP_OK2 "@0\n@0\n"
#define RESP_OK_1ARG "@1:%d\n"

#define DIR_FORWARD 2
#define DIR_BACKWARD 3

#define SPEED_ERROR 0.1f
#define THROTTLE_ERROR 0.25f

#define MIN_SPEED 300
#define MAX_SPEED 3600
#define SPEED_STEP 300

#define ONE_SECOND 1*1000*1000
#define SETTLE_TIME_US 500*1000
#define BREAK_TIME_US 500*1000
#define BREAK_CHECK_TIME_US 100*1000

#define SERVO_SETTLE_TIME_US 1*1000*1000


#define TARGET_SPEED_50 2200
#define TARGET_SPEED_50_ERR 200

//const char echo_cmd_str[] = "$e:01234567890123456789012345\n";
//const char echo_cmd_resp_str[] = "@0:01234567890123456789012345\n";

int fd;
char uart_buf[READ_BUF_SIZE];

int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        if (tcgetattr (fd, &tty) != 0)
        {
                fprintf(stderr, "error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                fprintf(stderr, "error %d from tcsetattr\n", errno);
                return -1;
        }
        return 0;
}


class MyAppTestSuite : public testing::Test
{
	void SetUp(){
		fd = open(PORTNAME, O_RDWR | O_NOCTTY | O_SYNC);
		EXPECT_TRUE(fd > 0);
		EXPECT_TRUE(set_interface_attribs (fd, BAUD_RATE, 0) == 0);
		// flush buffer
		int ret = read(fd, uart_buf, sizeof(uart_buf));
		EXPECT_TRUE(ret >= 0);
		Break();
		usleep(SETTLE_TIME_US);
	}

	void TearDown() {
		SetMotors(0, 0);
	}

public:

	void ReadResponse(const char* exp_resp) {
		int ret;
		size_t size = 0;
		while (1) {
			ret = read(fd, &uart_buf[size], sizeof(uart_buf) - size);
			if (ret < 0)
				break;
			size += ret;
			if (size >= strlen(exp_resp))
				break;
			usleep(10*1000);
		}
		uart_buf[size] = 0;
		//printf("size %lu: %s\n", size, uart_buf);
		EXPECT_TRUE(size == strlen(exp_resp) && strcmp(uart_buf, exp_resp) == 0);
	}

	void SetMotors(int vel_l, int vel_r) {
		int ret;

		sprintf(uart_buf, CMD_MOTOR_SET_SPEED, vel_l, vel_r);
		ret = write(fd, uart_buf, strlen(uart_buf));
		EXPECT_EQ(ret, strlen(uart_buf));
		ReadResponse(RESP_OK2);
	}

	void SetServo(int id, uint8_t pulse) {
		int ret;
		char buf[30];

		sprintf(uart_buf, CMD_SERVO_SET, id, pulse);
		ret = write(fd, uart_buf, strlen(uart_buf));
		EXPECT_EQ(ret, strlen(uart_buf));
		ReadResponse(RESP_OK);

		sprintf(uart_buf, CMD_SERVO_GET, id);
		ret = write(fd, uart_buf, strlen(uart_buf));
		EXPECT_EQ(ret, strlen(uart_buf));
		sprintf(buf, RESP_OK_1ARG, pulse);
		ReadResponse(buf);
	}

	void SetThrottle(int dir_l, int thr_l, int dir_r, int thr_r) {
		int ret;
		sprintf(uart_buf, CMD_MOTOR_SET_THROTTLE, dir_l, thr_l, dir_r, thr_r);
		ret = write(fd, uart_buf, strlen(uart_buf));
		EXPECT_EQ(ret, strlen(uart_buf));
		ReadResponse(RESP_OK2);
	}

	void Break() {
		SetThrottle(1, 0, 1, 0);
		usleep(BREAK_TIME_US);
	}

	void ReadMotors(int* pos_l, int* pos_r) {
		int ret;
		int status_l, status_r;
		ret = write(fd, CMD_MOTOR_READ_POS, strlen(CMD_MOTOR_READ_POS));
		EXPECT_EQ(ret, strlen(CMD_MOTOR_READ_POS));
		ret = read(fd, uart_buf, sizeof(uart_buf));
		if (ret > 0)
			uart_buf[ret] = 0;
		//printf("ret %d: %s\n", ret, uart_buf);
		EXPECT_TRUE(ret >= 7);
		ret = sscanf(uart_buf, "@%d:%d\n@%d:%d\n", &status_l, pos_l, &status_r, pos_r);
		EXPECT_TRUE(ret == 4);
		EXPECT_TRUE(status_l == 1 && status_r == 1);
	}
	
	void SpeedTest(int speed_l, int speed_r) {
		int save_pos_l, save_pos_r;
		int pos_l, pos_r;
		int real_speed_l, real_speed_r;
		float speed_error_l = 0.0f, speed_error_r = 0.0f;
		SetMotors(speed_l, speed_r);
		usleep(SETTLE_TIME_US);
		ReadMotors(&save_pos_l, &save_pos_r);
		usleep(ONE_SECOND);
		ReadMotors(&pos_l, &pos_r);

		real_speed_l = pos_l - save_pos_l;
		real_speed_r = pos_r - save_pos_r;
		if (speed_l) {
			speed_error_l = float(real_speed_l - speed_l) / speed_l;
			EXPECT_TRUE( fabs(speed_error_l) < SPEED_ERROR);
		} else 
			EXPECT_TRUE(pos_l == save_pos_l);

		if (speed_r) {
			speed_error_r = float(real_speed_r - speed_r) / speed_r;
			EXPECT_TRUE( fabs(speed_error_r) < SPEED_ERROR);
		} else 
			EXPECT_TRUE(pos_r == save_pos_r);
		printf("l = %5d, err = %6.2f%%, r = %5d, err = %6.2f%%\n", real_speed_l, speed_error_l * 100, real_speed_r, speed_error_r * 100);
		
	}
};

TEST_F(MyAppTestSuite, speed_pos_50Hz) {
	int pos_l, pos_r;
	for (int i = 0; i < 50 * 5; i++) {
		ReadMotors(&pos_l, &pos_r);
		SetMotors(300, 300);
		usleep(20 * 1000);
	}
}

TEST_F(MyAppTestSuite, speed) {
	int speed;
	SpeedTest(MIN_SPEED, 0);
	SpeedTest(0, MIN_SPEED);
	SpeedTest(-MIN_SPEED, 0);
	SpeedTest(0, -MIN_SPEED);
	for (speed = MIN_SPEED; speed <= MAX_SPEED; speed += SPEED_STEP) {
		SpeedTest(speed, speed);
	}
	SetMotors(0, 0); // fix driver to reset speed when breaking?
	Break();
	for (speed = -MIN_SPEED; speed >= -MAX_SPEED; speed -= SPEED_STEP) {
		SpeedTest(speed, speed);
	}
}

TEST_F(MyAppTestSuite, breaks) {
	int save_pos_l, save_pos_r;
	int pos_l, pos_r;
	int dir;

	for (dir = DIR_FORWARD; dir <= DIR_BACKWARD; dir++) {
		ReadMotors(&save_pos_l, &save_pos_r);
		SetThrottle(dir, 100, dir, 100);
		usleep(SETTLE_TIME_US);

		ReadMotors(&pos_l, &pos_r);
		if (dir == DIR_FORWARD) {
			EXPECT_TRUE(pos_l > save_pos_l);
			EXPECT_TRUE(pos_r > save_pos_r);
		} else {
			EXPECT_TRUE(pos_l < save_pos_l);
			EXPECT_TRUE(pos_r < save_pos_r);
		}
		Break();
		ReadMotors(&save_pos_l, &save_pos_r);
		usleep(BREAK_CHECK_TIME_US);
		ReadMotors(&pos_l, &pos_r);
		EXPECT_TRUE(pos_l == save_pos_l);
		EXPECT_TRUE(pos_r == save_pos_r);
	}
}

TEST_F(MyAppTestSuite, throttle) {
	int save_pos_l, save_pos_r;
	int pos_l, pos_r;
	int speed_l, speed_r;
	int dir;
	char modes[] = {'l', 'r', 'x'};

	for (dir = DIR_FORWARD; dir <= DIR_BACKWARD; dir++) {
		for (int i = 0; i < 3; i++) {
			Break();
			printf("Test dir %d, mode %c\n", dir, modes[i]);
			if (modes[i] == 'l')
				SetThrottle(dir, 50, dir, 0);
			else if (modes[i] == 'r')
				SetThrottle(dir, 0, dir, 50);
			else
				SetThrottle(dir, 50, dir, 50);

			usleep(SETTLE_TIME_US);
			ReadMotors(&save_pos_l, &save_pos_r);

			usleep(ONE_SECOND);
			ReadMotors(&pos_l, &pos_r);
			speed_l = pos_l - save_pos_l;
			speed_r = pos_r - save_pos_r;

			printf("l50 = %5d, r50 = %5d\n", speed_l, speed_r);
			if (dir == DIR_FORWARD) {
				if (modes[i] != 'l')
					EXPECT_TRUE(speed_r > 0 && abs(speed_r - TARGET_SPEED_50) < TARGET_SPEED_50_ERR);
				else
					EXPECT_TRUE(speed_r == 0);
				if (modes[i] != 'r')
					EXPECT_TRUE(speed_l > 0 && abs(speed_l - TARGET_SPEED_50) < TARGET_SPEED_50_ERR);
				else
					EXPECT_TRUE(speed_l == 0);
			} else {
				if (modes[i] != 'l')
					EXPECT_TRUE(pos_r < save_pos_r && abs(speed_r + TARGET_SPEED_50) < TARGET_SPEED_50_ERR);
				else
					EXPECT_TRUE(pos_r == save_pos_r);
				if (modes[i] != 'r')
					EXPECT_TRUE(pos_l < save_pos_l && abs(speed_l + TARGET_SPEED_50) < TARGET_SPEED_50_ERR);
				else
					EXPECT_TRUE(pos_l == save_pos_l);
			}
			
		}
	}

}

TEST_F(MyAppTestSuite, dummy) {
}

TEST_F(MyAppTestSuite, servo) {
	SetServo(0, 0);
	usleep(SERVO_SETTLE_TIME_US);
	SetServo(0, 100);
	usleep(SERVO_SETTLE_TIME_US);
	SetServo(0, 50);
}

TEST_F(MyAppTestSuite, echo_stress) {
	char echo_cmd_str[READ_BUF_SIZE];
	for (int i = 0; i < 100; i++) {
		sprintf(echo_cmd_str, CMD_ECHO_PREFIX);
		for (int j = 3 ; j < READ_BUF_SIZE - 2; j++)
			echo_cmd_str[j] = 'a' + (rand() % 26);
		echo_cmd_str[READ_BUF_SIZE - 2] = '\n';
		echo_cmd_str[READ_BUF_SIZE - 1] = 0;
		int ret = write(fd, echo_cmd_str, strlen(echo_cmd_str));
		EXPECT_EQ(ret, strlen(echo_cmd_str));
		echo_cmd_str[0] = '@';
		echo_cmd_str[1] = '0';
		ReadResponse(echo_cmd_str);
	}
}

