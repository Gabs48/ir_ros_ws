#ifndef SERIAL_H
#define SERIAL_H

#include <deque>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#ifdef POSIX
#include <termios.h>
#endif

using namespace std;

class Serial {
public:

	/*
	 * Constructor
	 */
	Serial(boost::asio::io_service& io_service, unsigned int baud, const string& device)
	: active_(true), io_service_(io_service), serialPort(io_service, device)
	{
		if (not serialPort.is_open()) {
			cerr << "Failed to open serial port\n";
			return;
		}
		boost::asio::serial_port_base::baud_rate baud_option(baud);
		serialPort.set_option(baud_option); // set the baud rate after the port has been opened
		read_start();
	}

	/*
	 * Pass the write data to the do_write function via the io service in the other thread
	 */
	void write(const char msg)
	{
		io_service_.post(boost::bind(&Serial::do_write, this, msg));
	}

	/*
	 * Call the do_close function via the io service in the other thread
	 */
	void close()
	{
		io_service_.post(boost::bind(&Serial::do_close, this, boost::system::error_code()));
	}

	/*
	 * Return true if the socket is still active
	 */
	bool active()
	{
		return active_;
	}

private:

	// Maximum amount of data to read in one operation
	static const int max_read_length = 512;

	/*
	 *  Start an asynchronous read and call read_complete when it completes or fails
	 */
	void read_start(void)
	{
		serialPort.async_read_some(boost::asio::buffer(read_msg_, max_read_length),
			boost::bind(&Serial::read_complete,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

	/*
	 * The asynchronous read operation has now completed or failed and returned an error
	 */
	void read_complete(const boost::system::error_code& error, size_t bytes_transferred)
	{
		string msg = "";
		if (!error) { // read completed, so process the data
			cout.write(read_msg_, bytes_transferred); // echo to standard output
			read_start(); // start waiting for another asynchronous read again
		} else {
			do_close(error);
		}
	}

	/*
	 * Callback to handle write call from outside this class
	 */
	void do_write(const char msg)
	{
		bool write_in_progress = !write_msgs_.empty(); // is there anything currently being written?
		write_msgs_.push_back(msg); // store in write buffer
		if (!write_in_progress) // if nothing is currently being written, then start
			write_start();
	}

	/*
	 * Start an asynchronous write and call write_complete when it completes or fails
	 */
	void write_start(void)
	{
		boost::asio::async_write(serialPort,
			boost::asio::buffer(&write_msgs_.front(), 1),
			boost::bind(&Serial::write_complete,
			this,
			boost::asio::placeholders::error));
	}

	/*
	 * The asynchronous read operation has now completed or failed and returned an error
	 */
	void write_complete(const boost::system::error_code& error)
	{
		if (!error) { // write completed, so send next write data
			write_msgs_.pop_front(); // remove the completed data
			if (!write_msgs_.empty()) // if there is anthing left to be written
				write_start(); // then start sending the next item in the buffer
		} else {
			do_close(error);
		}
	}

	void do_close(const boost::system::error_code& error)
	{
		if (error == boost::asio::error::operation_aborted) // if this call is the result of a timer cancel()
			return; // ignore it because the connection cancelled the timer
		if (error)
			cerr << "Error: " << error.message() << endl; // show the error message
		else
			cout << "Error: Connection did not succeed.\n";

		cout << "Press Enter to exit\n";
		serialPort.close();
		active_ = false;
	}

private:
	// Remains true while this object is still operating
	bool active_;

	// The main IO service that runs this connection
	boost::asio::io_service& io_service_;

	// The serial port this instance is connected to
	boost::asio::serial_port serialPort;

	// Data read from the socket
	char read_msg_[max_read_length];

	// Buffered write data
	deque<char> write_msgs_;
};


class Imu {
public:
	Imu() {}
	~Imu() {}

	void init() {
		// on Unix POSIX based systems, turn off line buffering of input, so cin.get() returns after every keypress
		// On other systems, you'll need to look for an equivalent
		#ifdef POSIX
			termios stored_settings;
			tcgetattr(0, &stored_settings);
			termios new_settings = stored_settings;
			new_settings.c_lflag &= (~ICANON);
			new_settings.c_lflag &= (~ISIG); // don't automatically handle control-C
			tcsetattr(0, TCSANOW, &new_settings);
		#endif

		try {
			// Create an instance of Serial
			io_service = new boost::asio::io_service();
			c = new Serial(*io_service, 115200, "/dev/ttyUSB0");

			// Create a boost thread and assign shell
			t = new boost::thread(boost::bind(&boost::asio::io_service::run, io_service));
		} catch (exception& e) {
			cerr << "Exception: " << e.what() << "\n";
		}
	}

	void close() {
		try {
			c->close(); // close the client connection
			t->join(); // wait for the IO service thread to close
		} catch (exception& e) {
			cerr << "Exception: " << e.what() << "\n";
		}
		#ifdef POSIX // restore default buffering of standard input
			tcsetattr(0, TCSANOW, &stored_settings);
		#endif
	}

private:

	boost::asio::io_service *io_service;
	Serial *c;
	boost::thread *t;
	void parse();

};


#endif // SERIAL_H
